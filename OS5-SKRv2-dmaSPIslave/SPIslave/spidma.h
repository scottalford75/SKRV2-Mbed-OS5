#ifndef SPIDMA_H
#define SPIDMA_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal_spi.h"

 
void initialiseSPI1slave();
void initialiseDMA();
void processPacket();

extern SPI_HandleTypeDef hspi1;
 
#ifdef __cplusplus
}
#endif

#endif