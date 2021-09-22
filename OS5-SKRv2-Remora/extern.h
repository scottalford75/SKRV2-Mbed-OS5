#ifndef EXTERN_H
#define EXTERN_H

#include "configuration.h"
#include "remora.h"

#include "lib/ArduinoJson6/ArduinoJson.h"
#include "thread/pruThread.h"


extern JsonObject module;

extern volatile bool PRUreset;

// unions for RX and TX data
extern volatile rxData_t rxData;
extern volatile txData_t txData;

// pointers to objects with global scope
extern pruThread* baseThread;
extern pruThread* servoThread;
extern pruThread* commsThread;

// pointers to data
extern volatile rxData_t*  ptrRxData;
extern volatile txData_t*  ptrTxData;
extern volatile int32_t*   ptrTxHeader;  
extern volatile bool*      ptrPRUreset;
extern volatile int32_t*   ptrJointFreqCmd[JOINTS];
extern volatile int32_t*   ptrJointFeedback[JOINTS];
extern volatile uint8_t*   ptrJointEnable;
extern volatile float*     ptrSetPoint[VARIABLES];
extern volatile float*     ptrProcessVariable[VARIABLES];
extern volatile uint8_t*   ptrInputs;
extern volatile uint8_t*   ptrOutputs;


#endif