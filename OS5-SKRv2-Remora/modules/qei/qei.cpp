#include "mbed.h"
#include "qei.h"

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/
void createQEI()
{
    const char* comment = module["Comment"];
    printf("%s\n",comment);

    int pv = module["PV[i]"];
    int dataBit = module["Data Bit"];
    const char* index = module["Enable Index"];

    printf("Creating QEI, hardware quadrature encoder interface\n");

    ptrProcessVariable[pv]  = &txData.processVariable[pv];
    ptrInputs = &txData.inputs;

    if (!strcmp(index,"True"))
    {
        printf("  Encoder has index\n");
        Module* qei = new QEI(*ptrProcessVariable[pv], *ptrInputs, dataBit);
        baseThread->registerModule(qei);
    }
    else
    {
        Module* qei = new QEI(*ptrProcessVariable[pv]);
        baseThread->registerModule(qei);
    }
}

/***********************************************************************
*                METHOD DEFINITIONS                                    *
************************************************************************/

QEI::QEI(volatile float &ptrEncoderCount) :
	ptrEncoderCount(&ptrEncoderCount),
    qeiIndex(NC)
{
    this->hasIndex = false;
    this->configQEI();
}

QEI::QEI(volatile float &ptrEncoderCount, volatile uint8_t &ptrData, int bitNumber) :
	ptrEncoderCount(&ptrEncoderCount),
    ptrData(&ptrData),
    bitNumber(bitNumber),
    qeiIndex(PE_13)
{
    this->hasIndex = true;
    this->indexDetected = false;
    this->indexPulse = 100;                             
	this->count = 0;								    
    this->indexCount = 0;
    this->oldIndexCount = 0;
    this->pulseCount = 0;                               
    this->mask = 1 << this->bitNumber;

    this->irq = EXTI15_10_IRQn;


    this->configQEI();

    qeiIndex.rise(callback(this, &QEI::interruptHandler));
    //NVIC_EnableIRQ(this->irq);
    HAL_NVIC_SetPriority(this->irq, 0, 0);
}


void QEI::interruptHandler()
{
    this->indexDetected = true;
    this->indexCount = this->getPosition();
}


uint32_t QEI::getPosition()
{
    return __HAL_TIM_GET_COUNTER(&htim);
}


void QEI::update()
{
    this->count = getPosition();

    if (this->hasIndex)                                     // we have an index pin
    {
        // handle index, index pulse and pulse count
        if (this->indexDetected && (this->pulseCount == 0))    // index interrupt occured: rising edge on index pulse
        {
            *(this->ptrEncoderCount) = this->indexCount;
            this->pulseCount = this->indexPulse;        
            *(this->ptrData) |= this->mask;                 // set bit in data source high
        }
        else if (this->pulseCount > 0)                      // maintain both index output and encoder count for the latch period
        {
            this->indexDetected = false;
            this->pulseCount--;                             // decrement the counter
        }
        else
        {
            *(this->ptrData) &= ~this->mask;                // set bit in data source low
            *(this->ptrEncoderCount) = this->count;         // update encoder count
        }
    }
    else
    {
        *(this->ptrEncoderCount) = this->count;             // update encoder count
    }
}

// reference https://os.mbed.com/users/gregeric/code/Nucleo_Hello_Encoder/

void QEI::configQEI()
{
    printf("  Configuring hardware QEI module\n");

    this->htim.Instance = TIM1;
    this->htim.Init.Prescaler = 0;
    this->htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    this->htim.Init.Period = 65535;
    this->htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    this->htim.Init.RepetitionCounter = 0;

    this->sConfig.EncoderMode = TIM_ENCODERMODE_TI12;

    this->sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    this->sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    this->sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    this->sConfig.IC1Filter = 0;

    this->sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    this->sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    this->sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    this->sConfig.IC2Filter = 0;

    if (HAL_TIM_Encoder_Init(&this->htim, &this->sConfig) != HAL_OK)
    {
        printf("Couldn't Init Encoder\r\n");
    }

    this->sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    this->sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&this->htim, &this->sMasterConfig);

    if (HAL_TIM_Encoder_Start(&this->htim, TIM_CHANNEL_2)!=HAL_OK)
    {
        printf("Couldn't Start Encoder\r\n");
    }
}


void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(htim_encoder->Instance==TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();

        __HAL_RCC_GPIOE_CLK_ENABLE();
        /**TIM1 GPIO Configuration
        PE9     ------> TIM1_CH1
        PE11     ------> TIM1_CH2
        */
        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    }
}

