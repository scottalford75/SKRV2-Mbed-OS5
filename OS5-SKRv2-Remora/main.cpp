
// MBED includes
#include "mbed.h"
#include <cstdio>
#include <cerrno>
#include <string> 
#include "FATFileSystem.h"


// Local includes
#include "configuration.h"
#include "remora.h"

#include "spidma.h"

//#include "SDIO/sdio_device.h"
#include "lib/SDIO/SDIOBlockDevice.h"
#include "lib/ArduinoJson6/ArduinoJson.h"

#include "drivers/pin/pin.h"
#include "drivers/SPI/RemoraSPI.h"

#include "thread/pruThread.h"
#include "thread/interrupt.h"

#include "modules/module.h"
#include "modules/debug/debug.h"
#include "modules/blink/blink.h"




SDIOBlockDevice blockDevice;
FATFileSystem fileSystem("fs");


// Watchdog
Watchdog& watchdog = Watchdog::get_instance();

// Json configuration file stuff
FILE *jsonFile;
string strJson;
DynamicJsonDocument doc(JSON_BUFF_SIZE);




/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream1;

// state machine
enum State {
    ST_SETUP = 0,
    ST_START,
    ST_IDLE,
    ST_RUNNING,
    ST_STOP,
    ST_RESET,
    ST_WDRESET
};


// unions for RX and TX data
//volatile rxData_t spiRxBuffer;  // this buffer is used to check for valid data before moveing it to rxData
volatile rxData_t rxData;
volatile txData_t txData;


// pointers to objects with global scope
pruThread* baseThread;
pruThread* servoThread;

// pointers to data
volatile rxData_t*  ptrRxData = &rxData;
volatile txData_t*  ptrTxData = &txData;


RemoraSPI spiSlave(ptrRxData, ptrTxData, SPI1, PA_4);

// Initialise the digital pins as outputs for Blinky
DigitalOut motEnable(PC_13);
//DigitalOut HE1(PB_4);

//InterruptIn slaveSelect(PA_4);



void TIM3_IRQHandler()
{
  if(TIM3->SR & TIM_SR_UIF) // if UIF flag is set
  {
    TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
    
    Interrupt::TIM3_Wrapper();
  }
}

void TIM10_IRQHandler()
{
  if(TIM10->SR & TIM_SR_UIF) // if UIF flag is set
  {
    TIM10->SR &= ~TIM_SR_UIF; // clear UIF flag
    
    Interrupt::TIM10_Wrapper();
  }
}


void readJsonConfig()
{
    printf("1. Reading json configuration file\n");

    // Try to mount the filesystem
    printf("Mounting the filesystem... ");
    fflush(stdout);
 
    int err = fileSystem.mount(&blockDevice);
    printf("%s\n", (err ? "Fail :(" : "OK"));
    if (err) {
        printf("No filesystem found... ");
        fflush(stdout);
     }

    // Open the config file
    printf("Opening \"/fs/config.txt\"... ");
    fflush(stdout);
    jsonFile = fopen("/fs/config.txt", "r+");
    printf("%s\n", (!jsonFile ? "Fail :(" : "OK"));

    fseek (jsonFile, 0, SEEK_END);
    int32_t length = ftell (jsonFile);
    fseek (jsonFile, 0, SEEK_SET);

    printf("Json config file lenght = %2d\n\r", length);

    strJson.reserve(length+1);

    while (!feof(jsonFile)) {
        int c = fgetc(jsonFile);
        strJson.push_back(c);
    }

    // Remove comments from next line to print out the JSON config file
    //printf("%s\n", strJson.c_str());

    printf("Closing \"/fs/config.txt\"... \n\r");
    fflush(stdout);
    fclose(jsonFile);
}



int main() {
    
    enum State currentState;
    enum State prevState;

    currentState = ST_SETUP;
    prevState = ST_RESET;

    printf("\nRemora PRU - Programmable Realtime Unit\n");

    watchdog.start(2000);

    while(1)
    {
      // the main loop does very little, keeping the Watchdog serviced and
      // resetting the rxData buffer if there is a loss of SPI commmunication
      // with LinuxCNC. Everything else is done via DMA and within the
      // two threads- Base and Servo threads that run the Modules.

    watchdog.kick();

    switch(currentState){
        case ST_SETUP:
            // do setup tasks
            if (currentState != prevState)
            {
                printf("## Entering SETUP state\n");
            }
            prevState = currentState;

            readJsonConfig();
            //setup();
            //loadModules();

            currentState = ST_START;
            break; 

        case ST_START:
            // do start tasks
            if (currentState != prevState)
            {
                printf("\n## Entering START state\n");
            }
            prevState = currentState;

            if (!threadsRunning)
            {
                // Start the threads
                printf("\nStarting the BASE thread\n");
                baseThread->startThread();
                
                printf("\nStarting the SERVO thread\n");
                servoThread->startThread();

                threadsRunning = true;

                // wait for threads to read IO before testing for PRUreset
                wait(1);
            }

            if (PRUreset)
            {
                // RPi outputs default is high until configured when LinuxCNC spiPRU component is started, PRUreset pin will be high
                // stay in start state until LinuxCNC is started
                currentState = ST_START;
            }
            else
            {
                currentState = ST_IDLE;
            }
            
            break;


        case ST_IDLE:
            // do something when idle
            if (currentState != prevState)
            {
                printf("\n## Entering IDLE state\n");
            }
            prevState = currentState;

            // check to see if there there has been SPI errors
            if (spiSlave.getError())
            {
                printf("SPI data error:\n");
                spiSlave.getError(false);
            }

            //wait for SPI data before changing to running state
            if (spiSlave.getStatus())
            {
                currentState = ST_RUNNING;
            }

            if (PRUreset) 
            {
                currentState = ST_WDRESET;
            }

            break;

        case ST_RUNNING:
            // do running tasks
            if (currentState != prevState)
            {
                printf("\n## Entering RUNNING state\n");
            }
            prevState = currentState;

            // check to see if there there has been SPI errors 
            if (spiSlave.getError())
            {
                printf("SPI data error:\n");
                spiSlave.getError(false);
            }
            
            if (spiSlave.getStatus())
            {
                // SPI data received by DMA
                resetCnt = 0;
                spiSlave.setStatus(false);
            }
            else
            {
                // no SPI data received by DMA
                resetCnt++;
            }

            if (resetCnt > SPI_ERR_MAX)
            {
                // reset threshold reached, reset the PRU
                printf("   SPI data error limit reached, resetting\n");
                resetCnt = 0;
                currentState = ST_RESET;
            }

            if (PRUreset) 
            {
                currentState = ST_WDRESET;
            }

            break;

        case ST_STOP:
            // do stop tasks
            if (currentState != prevState)
            {
                printf("\n## Entering STOP state\n");
            }
            prevState = currentState;


            currentState = ST_STOP;
            break;

        case ST_RESET:
            // do reset tasks
            if (currentState != prevState)
            {
                printf("\n## Entering RESET state\n");
            }
            prevState = currentState;

            // set all of the rxData buffer to 0
            // rxData.rxBuffer is volatile so need to do this the long way. memset cannot be used for volatile
            printf("   Resetting rxBuffer\n");
            {
                int n = sizeof(rxData.rxBuffer);
                while(n-- > 0)
                {
                    rxData.rxBuffer[n] = 0;
                }
            }

            currentState = ST_IDLE;
            break;

        case ST_WDRESET:
            // do a watch dog reset
            printf("\n## Entering WDRESET state\n");

            // force a watchdog reset by looping here
            while(1){}

            break;
      }

    wait(LOOP_TIME);
    }
}
