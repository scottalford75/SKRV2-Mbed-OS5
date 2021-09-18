#include "interrupt.h"
#include "stm32f4xx_hal.h"

#include <cstdio>

// Define the vector table, it is only declared in the class declaration
Interrupt* Interrupt::ISRVectorTable[] = {0};

// Constructor
Interrupt::Interrupt(void){}


// Methods

void Interrupt::Register(int interruptNumber, Interrupt* intThisPtr)
{
	printf("Registering interrupt for interrupt number = %d\n", interruptNumber);
	ISRVectorTable[interruptNumber] = intThisPtr;
}

void Interrupt::TIM3_Wrapper(void)
{
	ISRVectorTable[TIM3_IRQn]->ISR_Handler();
}

void Interrupt::TIM10_Wrapper(void)
{
	ISRVectorTable[TIM1_UP_TIM10_IRQn]->ISR_Handler();
}