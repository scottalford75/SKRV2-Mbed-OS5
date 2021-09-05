
#include "mbed.h"
#include "platform/mbed_thread.h"

// Local includes
#include "configuration.h"
#include "remora.h"
#include "spidma.h"



// Blinking rate in milliseconds
#define BLINKING_RATE_MS  500


// unions for RX and TX data
volatile rxData_t spiRxBuffer;  // this buffer is used to check for valid data before moveing it to rxData
volatile rxData_t rxData;
volatile txData_t txData;


// Initialise the digital pins as outputs for Blinky
DigitalOut motEnable(PC_13);
DigitalOut HE1(PB_4);

InterruptIn slaveSelect(PA_4);

int counter;


void processPacket()
{
    counter++;
}


int main()
{

    printf("SKRV2 is alive\n\r");
    motEnable = 1;
    int oldcnt;

    txData.header = PRU_DATA;


    // initialise SPI slave with DMA
    spi_init();

    slaveSelect.rise(&processPacket);

    // start the SPI transmit and recieve
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)txData.txBuffer, (uint8_t *)rxData.rxBuffer, SPI_BUFF_SIZE);

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
