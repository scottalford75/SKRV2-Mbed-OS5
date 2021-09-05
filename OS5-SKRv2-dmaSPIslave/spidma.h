#ifndef SPIDMA_H
#define SPIDMA_H
 
#include "stm32f4xx_hal_spi.h"
 
void initialiseSPI1slave();
void initialiseDMA();
void DMA2_Stream0_IRQHandler();
void DMA2_Stream3_IRQHandler();

extern SPI_HandleTypeDef hspi1;
 
#endif