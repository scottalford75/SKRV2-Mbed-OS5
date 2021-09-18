#include "mbed.h"
#include "RemoraSPI.h"

#include "stm32f4xx_hal.h"

RemoraSPI::RemoraSPI(volatile rxData_t* ptrRxData, volatile txData_t* ptrTxData, SPI_TypeDef* spiType, PinName interruptPin) :
    ptrRxData(ptrRxData),
    ptrTxData(ptrTxData),
    spiType(spiType),
    slaveSelect(interruptPin)
{
    this->spiHandle.Instance = this->spiType;

    slaveSelect.rise(callback(this, &RemoraSPI::processPacket));

    printf("ptrRxData = %p\n", ptrRxData);
    ptrRxBuffer = &spiRxBuffer;
}



void RemoraSPI::init()
{
    if(spiHandle.Instance == SPI1)
    {
        printf("Initialising SPI1 slave\n");

        GPIO_InitTypeDef GPIO_InitStruct;

        /**SPI1 GPIO Configuration
        PA4     ------> SPI1_NSS
        PA5     ------> SPI1_SCK
        PA6     ------> SPI1_MISO
        PA7     ------> SPI1_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        __HAL_RCC_SPI1_CLK_ENABLE();

        this->spiHandle.Init.Mode           = SPI_MODE_SLAVE;
        this->spiHandle.Init.Direction      = SPI_DIRECTION_2LINES;
        this->spiHandle.Init.DataSize       = SPI_DATASIZE_8BIT;
        this->spiHandle.Init.CLKPolarity    = SPI_POLARITY_LOW;
        this->spiHandle.Init.CLKPhase       = SPI_PHASE_1EDGE;
        this->spiHandle.Init.NSS            = SPI_NSS_HARD_INPUT;
        this->spiHandle.Init.FirstBit       = SPI_FIRSTBIT_MSB;
        this->spiHandle.Init.TIMode         = SPI_TIMODE_DISABLE;
        this->spiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        this->spiHandle.Init.CRCPolynomial  = 10;

        HAL_SPI_Init(&this->spiHandle);


        printf("Initialising DMA for SPI\n");

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        this->hdma_spi_tx.Instance                   = DMA2_Stream3;
        this->hdma_spi_tx.Init.Channel               = DMA_CHANNEL_3;
        this->hdma_spi_tx.Init.Direction             = DMA_MEMORY_TO_PERIPH;
        this->hdma_spi_tx.Init.PeriphInc             = DMA_PINC_DISABLE;
        this->hdma_spi_tx.Init.MemInc                = DMA_MINC_ENABLE;
        this->hdma_spi_tx.Init.PeriphDataAlignment   = DMA_PDATAALIGN_BYTE;
        this->hdma_spi_tx.Init.MemDataAlignment      = DMA_MDATAALIGN_BYTE;
        this->hdma_spi_tx.Init.Mode                  = DMA_CIRCULAR;
        //this->hdma_spi_tx.Init.Mode                  = DMA_NORMAL;
        this->hdma_spi_tx.Init.Priority              = DMA_PRIORITY_VERY_HIGH;
        this->hdma_spi_tx.Init.FIFOMode              = DMA_FIFOMODE_DISABLE;
        
        HAL_DMA_Init(&this->hdma_spi_tx);

        __HAL_LINKDMA(&this->spiHandle, hdmatx, this->hdma_spi_tx);

        HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
        ///NVIC_SetVector(DMA2_Stream3_IRQn, (uint32_t)&DMA2_Stream3_IRQHandler);
        //HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

        this->hdma_spi_rx.Instance                   = DMA2_Stream0;
        this->hdma_spi_rx.Init.Channel               = DMA_CHANNEL_3;
        this->hdma_spi_rx.Init.Direction             = DMA_PERIPH_TO_MEMORY;
        this->hdma_spi_rx.Init.PeriphInc             = DMA_PINC_DISABLE;
        this->hdma_spi_rx.Init.MemInc                = DMA_MINC_ENABLE;
        this->hdma_spi_rx.Init.PeriphDataAlignment   = DMA_PDATAALIGN_BYTE;
        this->hdma_spi_rx.Init.MemDataAlignment      = DMA_MDATAALIGN_BYTE;
        this->hdma_spi_rx.Init.Mode                  = DMA_CIRCULAR;
        //this->hdma_spi_rx.Init.Mode                  = DMA_NORMAL;
        this->hdma_spi_rx.Init.Priority              = DMA_PRIORITY_VERY_HIGH;
        this->hdma_spi_rx.Init.FIFOMode              = DMA_FIFOMODE_DISABLE;

        HAL_DMA_Init(&this->hdma_spi_rx);

        __HAL_LINKDMA(&this->spiHandle,hdmarx,this->hdma_spi_rx);

        HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
        //NVIC_SetVector(DMA2_Stream0_IRQn, (uint32_t)&DMA2_Stream0_IRQHandler);
        //HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

        
        this->hdma_memtomem_dma2_stream1.Instance                 = DMA2_Stream1;
        this->hdma_memtomem_dma2_stream1.Init.Channel             = DMA_CHANNEL_0;
        this->hdma_memtomem_dma2_stream1.Init.Direction           = DMA_MEMORY_TO_MEMORY;
        this->hdma_memtomem_dma2_stream1.Init.PeriphInc           = DMA_PINC_ENABLE;
        this->hdma_memtomem_dma2_stream1.Init.MemInc              = DMA_MINC_ENABLE;
        this->hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        this->hdma_memtomem_dma2_stream1.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        this->hdma_memtomem_dma2_stream1.Init.Mode                = DMA_NORMAL;
        this->hdma_memtomem_dma2_stream1.Init.Priority            = DMA_PRIORITY_LOW;
        this->hdma_memtomem_dma2_stream1.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
        this->hdma_memtomem_dma2_stream1.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
        this->hdma_memtomem_dma2_stream1.Init.MemBurst            = DMA_MBURST_SINGLE;
        this->hdma_memtomem_dma2_stream1.Init.PeriphBurst         = DMA_PBURST_SINGLE;

        HAL_DMA_Init(&this->hdma_memtomem_dma2_stream1);

    }
}

void RemoraSPI::start()
{
    this->ptrTxData->header = PRU_DATA;
    HAL_SPI_TransmitReceive_DMA(&this->spiHandle, (uint8_t *)this->ptrTxData->txBuffer, (uint8_t *)this->spiRxBuffer.rxBuffer, SPI_BUFF_SIZE);
}

void RemoraSPI::processPacket()
{
    switch (this->spiRxBuffer.header)
    {
      case PRU_READ:
        this->SPIdata = true;
        this->rejectCnt = 0;
        // READ so do nothing with the received data
        break;

      case PRU_WRITE:
        this->SPIdata = true;
        this->rejectCnt = 0;
        // we've got a good WRITE header, move the data to rxData
        //HAL_DMA_Start(&hdma_memtomem_dma2_stream1, (uint32_t)&this->spiRxBuffer.rxBuffer, (uint32_t)&this->rxData->rxBuffer, SPI_BUFF_SIZE);
        //HAL_DMA_Start(&hdma_memtomem_dma2_stream1, (uint32_t)this->ptrRxBuffer, (uint32_t)this->ptrRxData, SPI_BUFF_SIZE);     
          
        for (int i = 0; i < SPI_BUFF_SIZE; i++)
        {
            //this->ptrRxData->rxBuffer[i] = this->spiRxBuffer.rxBuffer[i];
            this->ptrRxData->rxBuffer[i] = this->ptrRxBuffer->rxBuffer[i];
        }

        break;

      default:
        this->rejectCnt++;
        if (this->rejectCnt > 5)
        {
            this->SPIdataError = true;
        }
        // reset SPI somehow
    }

    HAL_SPI_TransmitReceive_DMA(&this->spiHandle, (uint8_t *)this->ptrTxData->txBuffer, (uint8_t *)this->spiRxBuffer.rxBuffer, SPI_BUFF_SIZE);
}


bool RemoraSPI::getStatus(void)
{
    return this->SPIdata;
}

void RemoraSPI::setStatus(bool status)
{
    this->SPIdata = status;
}

bool RemoraSPI::getError(void)
{
    return this->SPIdataError;
}

void RemoraSPI::setError(bool error)
{
    this->SPIdataError = error;
}