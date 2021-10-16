#ifndef QEI_H
#define QEI_H

#include "mbed.h"
#include <cstdint>

#include "modules/module.h"
#include "stm32f4xx_hal.h"

#include "extern.h"

void createQEI(void);

class QEI : public Module
{

	private:

        TIM_HandleTypeDef       htim;
        TIM_Encoder_InitTypeDef sConfig  = {0};
        TIM_MasterConfigTypeDef sMasterConfig  = {0};

        InterruptIn             qeiIndex;
        IRQn_Type 		        irq;

        bool                    hasIndex;
        bool                    indexDetected;
        volatile uint8_t*       ptrData; 	// pointer to the data source
		int                     bitNumber;				// location in the data source
        int                     mask;

		volatile float*         ptrEncoderCount; 	// pointer to the data source

        int32_t                 count;
        int32_t                 indexCount;
        int32_t                 oldIndexCount;
        int8_t                  indexPulse;
        int8_t                  pulseCount;

        void interruptHandler();

	public:

        QEI(volatile float&);                           // for channel A & B on BTT SKR2 pins PE_9 and PE_11
        QEI(volatile float&, volatile uint8_t&, int);   // For channels A & B, and index on BTT SKR2 pin PE_13

        void configQEI(void);
        uint32_t getPosition(void);

		virtual void update(void);
};

#endif
