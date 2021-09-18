
#include "mbed.h"
#include "stm32f4xx_hal.h"

//#include "configuration.h"
//#include "remora.h"

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_memtomem_dma2_stream1;


void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi1_rx);
}


void DMA2_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi1_tx);
}


void initialiseSPI1slave(void)
{
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

    hspi1.Instance            = SPI1;
    hspi1.Init.Mode           = SPI_MODE_SLAVE;
    hspi1.Init.Direction      = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize       = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity    = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase       = SPI_PHASE_1EDGE;
    hspi1.Init.NSS            = SPI_NSS_HARD_INPUT;
    hspi1.Init.FirstBit       = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode         = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial  = 10;

    HAL_SPI_Init(&hspi1);
}


void initialiseDMA(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_spi1_tx.Instance                   = DMA2_Stream3;
    hdma_spi1_tx.Init.Channel               = DMA_CHANNEL_3;
    hdma_spi1_tx.Init.Direction             = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc             = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc                = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment   = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment      = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode                  = DMA_CIRCULAR;
    //hdma_spi1_tx.Init.Mode                  = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority              = DMA_PRIORITY_VERY_HIGH;
    hdma_spi1_tx.Init.FIFOMode              = DMA_FIFOMODE_DISABLE;
    
    HAL_DMA_Init(&hdma_spi1_tx);

    __HAL_LINKDMA(&hspi1, hdmatx, hdma_spi1_tx);

    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
    NVIC_SetVector(DMA2_Stream3_IRQn, (uint32_t)&DMA2_Stream3_IRQHandler);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

    hdma_spi1_rx.Instance                   = DMA2_Stream0;
    hdma_spi1_rx.Init.Channel               = DMA_CHANNEL_3;
    hdma_spi1_rx.Init.Direction             = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc             = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc                = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment   = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment      = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode                  = DMA_CIRCULAR;
    //hdma_spi1_rx.Init.Mode                  = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority              = DMA_PRIORITY_VERY_HIGH;
    hdma_spi1_rx.Init.FIFOMode              = DMA_FIFOMODE_DISABLE;

    HAL_DMA_Init(&hdma_spi1_rx);

    __HAL_LINKDMA(&hspi1,hdmarx,hdma_spi1_rx);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    NVIC_SetVector(DMA2_Stream0_IRQn, (uint32_t)&DMA2_Stream0_IRQHandler);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    
    hdma_memtomem_dma2_stream1.Instance                 = DMA2_Stream1;
    hdma_memtomem_dma2_stream1.Init.Channel             = DMA_CHANNEL_0;
    hdma_memtomem_dma2_stream1.Init.Direction           = DMA_MEMORY_TO_MEMORY;
    hdma_memtomem_dma2_stream1.Init.PeriphInc           = DMA_PINC_ENABLE;
    hdma_memtomem_dma2_stream1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_memtomem_dma2_stream1.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_memtomem_dma2_stream1.Init.Mode                = DMA_NORMAL;
    hdma_memtomem_dma2_stream1.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_memtomem_dma2_stream1.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_memtomem_dma2_stream1.Init.FIFOThreshold        = DMA_FIFO_THRESHOLD_FULL;
    hdma_memtomem_dma2_stream1.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_memtomem_dma2_stream1.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    HAL_DMA_Init(&hdma_memtomem_dma2_stream1);
}

