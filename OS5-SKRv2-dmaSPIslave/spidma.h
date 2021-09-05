#ifndef SPIDMA_H
#define SPIDMA_H
 
#include "stm32f4xx_hal_spi.h"
 
void spi_init();

extern SPI_HandleTypeDef hspi1;
 
#endif