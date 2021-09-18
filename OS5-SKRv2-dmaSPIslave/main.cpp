
#include "mbed.h"
#include "platform/mbed_thread.h"

// Local includes
#include "configuration.h"
#include "remora.h"
#include "SPIdmaSlave.h"



// Blinking rate in milliseconds
#define BLINKING_RATE_MS  500

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

volatile uint8_t rejectCnt;

// boolean
static volatile bool SPIdata;
static volatile bool SPIdataError;


// pointers to objects with global scope


// unions for RX and TX data
volatile rxData_t spiRxBuffer;  // this buffer is used to check for valid data before moveing it to rxData
volatile rxData_t rxData;
volatile txData_t txData;


// Initialise the digital pins as outputs for Blinky
DigitalOut motEnable(PC_13);
DigitalOut HE1(PB_4);

SPIdmaSlave spiSlave();
InterruptIn slaveSelect(PA_4);

int counter;

void processPacket()
{
    counter++;

    switch (spiRxBuffer.header)
    {
      case PRU_READ:
        SPIdata = true;
        rejectCnt = 0;
        // READ so do nothing with the received data
        break;

      case PRU_WRITE:
        SPIdata = true;
        rejectCnt = 0;
        // we've got a good WRITE header, move the data to rxData
        HAL_DMA_Start(&hdma_memtomem_dma2_stream1, (uint32_t)&spiRxBuffer, (uint32_t)&rxData, SPI_BUFF_SIZE);
        break;

      default:
        rejectCnt++;
        if (rejectCnt > 5)
        {
            SPIdataError = true;
        }
        // reset SPI somehow
    }
/*
    HAL_DMA_Abort(&hdma_spi1_tx); 
    HAL_DMA_Abort(&hdma_spi1_rx); 
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();
    initialiseSPI1slave();
    initialiseDMA();
*/
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)txData.txBuffer, (uint8_t *)spiRxBuffer.rxBuffer, SPI_BUFF_SIZE);

}





int main()
{

    printf("SKRV2 is alive\n\r");
    motEnable = 1;
    int oldcnt;

    txData.header = PRU_DATA;


    // initialise SPI slave and DMA
    //initialiseSPI1slave();
    //initialiseDMA();
    spiSlave.init();
    spiSlave.deinit();

    slaveSelect.rise(&processPacket);

    // start the SPI transmit and recieve
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)txData.txBuffer, (uint8_t *)spiRxBuffer.rxBuffer, SPI_BUFF_SIZE);

    while (true) {
        HE1 = !HE1;

        if (oldcnt != counter)
        {
            oldcnt = counter;
            printf("Count so far: %d\n", counter);
            printf("spiRxBuffer header = %x\n", spiRxBuffer.header);
            printf("rx data header = %x\n", rxData.header);
        }

        thread_sleep_for(BLINKING_RATE_MS);
    }
}
