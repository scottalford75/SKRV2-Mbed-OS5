#ifndef REMORA_H
#define REMORA_H

typedef union
{
  // this allow structured access to the incoming SPI data without having to move it
  struct
  {
    uint8_t rxBuffer[SPI_BUFF_SIZE];
  };
  struct
  {
    int32_t header;
    volatile int32_t jointFreqCmd[JOINTS]; 	// Base thread commands ?? - basically motion
    float setPoint[VARIABLES];		  // Servo thread commands ?? - temperature SP, PWM etc
    uint8_t jointEnable;
    uint8_t outputs;
    uint8_t spare0;
    uint8_t spare1;
  };
} rxData_t;

extern volatile rxData_t rxData;
extern volatile rxData_t spiRxBuffer;


typedef union
{
  // this allow structured access to the out going SPI data without having to move it
  struct
  {
    uint8_t txBuffer[SPI_BUFF_SIZE];
  };
  struct
  {
    int32_t header;
    int32_t jointFeedback[JOINTS];	  // Base thread feedback ??
    float processVariable[VARIABLES];		     // Servo thread feedback ??
	uint8_t inputs;
  };
} txData_t;

extern volatile txData_t txData;

#endif
