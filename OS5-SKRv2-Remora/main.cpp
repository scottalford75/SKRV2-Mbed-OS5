
// MBED includes
#include "mbed.h"
#include <cstdio>
#include <cerrno>
#include <string> 
#include "FATFileSystem.h"


// Local includes
#include "configuration.h"
#include "remora.h"

#include "spidma.h"

//#include "SDIO/sdio_device.h"
#include "lib/SDIO/SDIOBlockDevice.h"
#include "lib/ArduinoJson6/ArduinoJson.h"

#include "drivers/pin/pin.h"
#include "drivers/SPI/RemoraSPI.h"

#include "thread/pruThread.h"
#include "thread/interrupt.h"

#include "modules/module.h"
#include "modules/blink/blink.h"
#include "modules/debug/debug.h"
#include "modules/digitalPin/digitalPin.h"
#include "modules/encoder/encoder.h"
#include "modules/eStop/eStop.h"
#include "modules/pwm/pwm.h"
#include "modules/rcservo/rcservo.h"
#include "modules/resetPin/resetPin.h"
#include "modules/stepgen/stepgen.h"
#include "modules/switch/switch.h"
#include "modules/temperature/temperature.h"
#include "modules/tmcStepper/tmcStepper.h"



/***********************************************************************
        STRUCTURES AND GLOBAL VARIABLES                       
************************************************************************/

// state machine
enum State {
    ST_SETUP = 0,
    ST_START,
    ST_IDLE,
    ST_RUNNING,
    ST_STOP,
    ST_RESET,
    ST_WDRESET
};

uint8_t resetCnt;

// boolean
static volatile bool PRUreset;
bool configError = false;
bool threadsRunning = false;

// unions for RX and TX data
//volatile rxData_t spiRxBuffer;  // this buffer is used to check for valid data before moveing it to rxData
volatile rxData_t rxData;
volatile txData_t txData;

// pointers to objects with global scope
pruThread* baseThread;
pruThread* servoThread;
pruThread* commsThread;

// pointers to data
volatile rxData_t*  ptrRxData = &rxData;
volatile txData_t*  ptrTxData = &txData;
volatile int32_t*   ptrTxHeader;  
volatile bool*      ptrPRUreset;
volatile int32_t*   ptrJointFreqCmd[JOINTS];
volatile int32_t*   ptrJointFeedback[JOINTS];
volatile uint8_t*   ptrJointEnable;
volatile float*     ptrSetPoint[VARIABLES];
volatile float*     ptrProcessVariable[VARIABLES];
volatile uint8_t*   ptrInputs;
volatile uint8_t*   ptrOutputs;



/***********************************************************************
        OBJECTS etc                                           
************************************************************************/

// SD card access
SDIOBlockDevice blockDevice;
FATFileSystem fileSystem("fs");

// SPI slave - RPi is the SPI master
RemoraSPI spiSlave(ptrRxData, ptrTxData, SPI1, PA_4);

// Watchdog
Watchdog& watchdog = Watchdog::get_instance();

// Json configuration file stuff
FILE *jsonFile;
string strJson;
DynamicJsonDocument doc(JSON_BUFF_SIZE);

DigitalOut motEnable(PC_13);        // *** REMOVE THIS***

/***********************************************************************
        INTERRUPT HANDLERS - add NVIC_SetVector etc to setup()
************************************************************************/


void TIM9_IRQHandler()
{
  if(TIM9->SR & TIM_SR_UIF) // if UIF flag is set
  {
    TIM9->SR &= ~TIM_SR_UIF; // clear UIF flag
    
    Interrupt::TIM9_Wrapper();
  }
}

void TIM10_IRQHandler()
{
  if(TIM10->SR & TIM_SR_UIF) // if UIF flag is set
  {
    TIM10->SR &= ~TIM_SR_UIF; // clear UIF flag
    
    Interrupt::TIM10_Wrapper();
  }
}

void TIM11_IRQHandler()
{
  if(TIM11->SR & TIM_SR_UIF) // if UIF flag is set
  {
    TIM11->SR &= ~TIM_SR_UIF; // clear UIF flag
    
    Interrupt::TIM11_Wrapper();
  }
}


/***********************************************************************
        ROUTINES
************************************************************************/

void readJsonConfig()
{
    printf("1. Reading json configuration file\n");

    // Try to mount the filesystem
    printf("Mounting the filesystem... ");
    fflush(stdout);
 
    int err = fileSystem.mount(&blockDevice);
    printf("%s\n", (err ? "Fail :(" : "OK"));
    if (err) {
        printf("No filesystem found... ");
        fflush(stdout);
     }

    // Open the config file
    printf("Opening \"/fs/config.txt\"... ");
    fflush(stdout);
    jsonFile = fopen("/fs/config.txt", "r+");
    printf("%s\n", (!jsonFile ? "Fail :(" : "OK"));

    fseek (jsonFile, 0, SEEK_END);
    int32_t length = ftell (jsonFile);
    fseek (jsonFile, 0, SEEK_SET);

    printf("Json config file lenght = %2d\n\r", length);

    strJson.reserve(length+1);

    while (!feof(jsonFile)) {
        int c = fgetc(jsonFile);
        strJson.push_back(c);
    }

    // Remove comments from next line to print out the JSON config file
    //printf("%s\n", strJson.c_str());

    printf("Closing \"/fs/config.txt\"... \n\r");
    fflush(stdout);
    fclose(jsonFile);
}


void setup()
{
    printf("\n2. Setting up SPI, DMA and threads\n");

    // deinitialise the SDIO device to avoid DMA issues with the SPI DMA Slave on the STM32F
    blockDevice.deinit();

    // initialise the SPI DMA Slave
    spiSlave.init();
    spiSlave.start();

    // Create the thread objects and set the interrupt vectors to RAM. This is needed
    // as we are using the SD bootloader that requires a different code starting
    // address. Also set interrupt priority with NVIC_SetPriority.

    baseThread = new pruThread(TIM9, TIM1_BRK_TIM9_IRQn, PRU_BASEFREQ);
    NVIC_SetVector(TIM1_BRK_TIM9_IRQn, (uint32_t)TIM9_IRQHandler);
    NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 2);

    servoThread = new pruThread(TIM10, TIM1_UP_TIM10_IRQn, PRU_SERVOFREQ);
    NVIC_SetVector(TIM1_UP_TIM10_IRQn, (uint32_t)TIM10_IRQHandler);
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 3);

    commsThread = new pruThread(TIM11, TIM1_TRG_COM_TIM11_IRQn, PRU_COMMSFREQ);
    NVIC_SetVector(TIM1_TRG_COM_TIM11_IRQn, (uint32_t)TIM11_IRQHandler);
    NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 4);


    // Other interrupt sources

}


void loadModules()
/*
{
    Module* debugOnS = new Debug("PE_5", 1);
    servoThread->registerModule(debugOnS);

    //Module* blink = new Blink("PB_4", PRU_SERVOFREQ, 1);
    //servoThread->registerModule(blink);

    ptrPRUreset = &PRUreset;
    printf("Make Reset Pin at pin PC_4\n");
    Module* resetPin = new ResetPin(*ptrPRUreset, "PC_4");
    servoThread->registerModule(resetPin);

    Module* debugOffS = new Debug("PE_5", 0);
    servoThread->registerModule(debugOffS);



    Module* debugOnB = new Debug("PE_4", 1);
    baseThread->registerModule(debugOnB);

    Module* blink = new Blink("PB_4", PRU_BASEFREQ, 1);
    baseThread->registerModule(blink);

    Module* debugOffB = new Debug("PE_4", 0);
    baseThread->registerModule(debugOffB);  
}*/

{
    printf("\n3. Parsing json configuration file\n");

    const char *json = strJson.c_str();

    // parse the json configuration file
    DeserializationError error = deserializeJson(doc, json);

    printf("Config deserialisation - ");

    switch (error.code())
    {
        case DeserializationError::Ok:
            printf("Deserialization succeeded\n");
            break;
        case DeserializationError::InvalidInput:
            printf("Invalid input!\n");
            configError = true;
            break;
        case DeserializationError::NoMemory:
            printf("Not enough memory\n");
            configError = true;
            break;
        default:
            printf("Deserialization failed\n");
            configError = true;
            break;
    }

    if (configError) return;

    JsonArray Modules = doc["Modules"];

    // create objects from json data
    for (JsonArray::iterator it=Modules.begin(); it!=Modules.end(); ++it)
    {
        JsonObject module = *it;
        
        const char* thread = module["Thread"];
        const char* type = module["Type"];

        if (!strcmp(thread,"Base"))
        {
            printf("\nBase thread object\n");

            if (!strcmp(type,"Stepgen"))
            {
                const char* comment = module["Comment"];
                printf("%s\n",comment);

                int joint = module["Joint Number"];
                const char* enable = module["Enable Pin"];
                const char* step = module["Step Pin"];
                const char* dir = module["Direction Pin"];

                // configure pointers to data source and feedback location
                ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
                ptrJointFeedback[joint] = &txData.jointFeedback[joint];
                ptrJointEnable = &rxData.jointEnable;

                // create the step generator, register it in the thread
                Module* stepgen = new Stepgen(PRU_BASEFREQ, joint, enable, step, dir, STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
                baseThread->registerModule(stepgen);
            }
            else if (!strcmp(type,"Encoder"))
            {
                const char* comment = module["Comment"];
                printf("%s\n",comment);
    
                int pv = module["PV[i]"];
                const char* pinA = module["ChA Pin"];
                const char* pinB = module["ChB Pin"];
                const char* pinI = module["Index Pin"];
                int dataBit = module["Data Bit"];
                const char* modifier = module["Modifier"];
            
                printf("Creating Quadrature Encoder at pins %s and %s\n", pinA, pinB);

                int mod;

                if (!strcmp(modifier,"Open Drain"))
                {
                    mod = OPENDRAIN;
                }
                else if (!strcmp(modifier,"Pull Up"))
                {
                    mod = PULLUP;
                }
                else if (!strcmp(modifier,"Pull Down"))
                {
                    mod = PULLDOWN;
                }
                else if (!strcmp(modifier,"Pull None"))
                {
                    mod = PULLNONE;
                }
                else
                {
                    mod = NONE;
                }
                
                ptrProcessVariable[pv]  = &txData.processVariable[pv];
                ptrInputs = &txData.inputs;

                if (pinI == nullptr)
                {
                    Module* encoder = new Encoder(*ptrProcessVariable[pv], pinA, pinB, mod);
                    baseThread->registerModule(encoder);
                }
                else
                {
                    printf("  Encoder has index at pin %s\n", pinI);
                    Module* encoder = new Encoder(*ptrProcessVariable[pv], *ptrInputs, dataBit, pinA, pinB, pinI, mod);
                    baseThread->registerModule(encoder);
                }

            }
            else if (!strcmp(type,"RCServo"))
            {
                const char* comment = module["Comment"];
                printf("%s\n",comment);
    
                int sp = module["SP[i]"];
                const char* pin = module["Servo Pin"];
            
                printf("Make RC Servo at pin %s\n", pin);
                
                ptrSetPoint[sp] = &rxData.setPoint[sp];

                // slow module with 10 hz update
                int updateHz = 10;
                Module* rcservo = new RCServo(*ptrSetPoint[sp], pin, PRU_BASEFREQ, updateHz);
                baseThread->registerModule(rcservo);

            }
        }
        else if (!strcmp(thread,"Servo"))
        {
            printf("\nServo thread object\n");

            if (!strcmp(type, "eStop"))
            {
                const char* comment = module["Comment"];
                printf("%s\n",comment);
    
                const char* pin = module["Pin"];
            
                ptrTxHeader = &txData.header;
    
                printf("Make eStop at pin %s\n", pin);

                Module* estop = new eStop(*ptrTxHeader, pin);
                servoThread->registerModule(estop);

            }
            else if (!strcmp(type, "Reset Pin"))
            {
                const char* comment = module["Comment"];
                printf("%s\n",comment);
    
                const char* pin = module["Pin"];
            
                ptrPRUreset = &PRUreset;
    
                printf("Make Reset Pin at pin %s\n", pin);

                Module* resetPin = new ResetPin(*ptrPRUreset, pin);
                servoThread->registerModule(resetPin);

            }
            else if (!strcmp(type, "Blink"))
            {
                const char* pin = module["Pin"];
                int frequency = module["Frequency"];
                
                printf("Make Blink at pin %s\n", pin);
                    
                Module* blink = new Blink(pin, PRU_SERVOFREQ, frequency);
                servoThread->registerModule(blink);
            }
            else if (!strcmp(type,"Digital Pin"))
            {
                const char* comment = module["Comment"];
                printf("%s\n",comment);
    
                const char* pin = module["Pin"];
                const char* mode = module["Mode"];
                const char* invert = module["Invert"];
                const char* modifier = module["Modifier"];
                int dataBit = module["Data Bit"];

                int mod;
                bool inv;

                if (!strcmp(modifier,"Open Drain"))
                {
                    mod = OPENDRAIN;
                }
                else if (!strcmp(modifier,"Pull Up"))
                {
                    mod = PULLUP;
                }
                else if (!strcmp(modifier,"Pull Down"))
                {
                    mod = PULLDOWN;
                }
                else if (!strcmp(modifier,"Pull None"))
                {
                    mod = PULLNONE;
                }
                else
                {
                    mod = NONE;
                }

                if (!strcmp(invert,"True"))
                {
                    inv = true;
                }
                else inv = false;

                ptrOutputs = &rxData.outputs;
                ptrInputs = &txData.inputs;
    
                printf("Make Digital %s at pin %s\n", mode, pin);
    
                if (!strcmp(mode,"Output"))
                {
                    //Module* digitalPin = new DigitalPin(*ptrOutputs, 1, pin, dataBit, invert);
                    Module* digitalPin = new DigitalPin(*ptrOutputs, 1, pin, dataBit, inv, mod);
                    servoThread->registerModule(digitalPin);
                }
                else if (!strcmp(mode,"Input"))
                {
                    //Module* digitalPin = new DigitalPin(*ptrInputs, 0, pin, dataBit, invert);
                    Module* digitalPin = new DigitalPin(*ptrInputs, 0, pin, dataBit, inv, mod);
                    servoThread->registerModule(digitalPin);
                }
                else
                {
                    printf("Error - incorrectly defined Digital Pin\n");
                }
            }
            else if (!strcmp(type,"PWM"))
            {
                const char* comment = module["Comment"];
                printf("%s\n",comment);
    
                int sp = module["SP[i]"];
                int pwmMax = module["PWM Max"];
                const char* pin = module["PWM Pin"];

                const char* hardware = module["Hardware PWM"];
                const char* variable = module["Variable Freq"];
                int period_sp = module["Period SP[i]"];
                int period = module["Period us"];
            
                printf("Make PWM at pin %s\n", pin);
                
                ptrSetPoint[sp] = &rxData.setPoint[sp];

                if (!strcmp(hardware,"True"))
                {
                    // Hardware PWM
                    if (!strcmp(variable,"True"))
                    {
                        // Variable frequency hardware PWM
                        ptrSetPoint[period_sp] = &rxData.setPoint[period_sp];

                        //Module* pwm = new HardwarePWM(*ptrSetPoint[period_sp], *ptrSetPoint[sp], period, pin);
                        //servoThread->registerModule(pwm);
                    }
                    else
                    {
                        // Fixed frequency hardware PWM
                        //Module* pwm = new HardwarePWM(*ptrSetPoint[sp], period, pin);
                        //servoThread->registerModule(pwm);
                    }
                }
                else
                {
                    // Software PWM
                    if (pwmMax != 0) // use configuration file value for pwmMax - useful for 12V on 24V systems
                    {
                        Module* pwm = new PWM(*ptrSetPoint[sp], pin, pwmMax);
                        servoThread->registerModule(pwm);
                    }
                    else // use default value of pwmMax
                    {
                        Module* pwm = new PWM(*ptrSetPoint[sp], pin);
                        servoThread->registerModule(pwm);
                    }
                }
            }
            else if (!strcmp(type,"Temperature"))
            { 
                printf("Make Temperature measurement object\n");
                const char* comment = module["Comment"];
                printf("%s\n",comment);

                int pv = module["PV[i]"];
                const char* sensor = module["Sensor"];

                ptrProcessVariable[pv]  = &txData.processVariable[pv];

                if (!strcmp(sensor, "Thermistor"))
                {
                    const char* pinSensor = module["Thermistor"]["Pin"];
                    float beta =  module["Thermistor"]["beta"];
                    int r0 = module["Thermistor"]["r0"];
                    int t0 = module["Thermistor"]["t0"];

                    // slow module with 1 hz update
                    int updateHz = 1;
                    Module* temperature = new Temperature(*ptrProcessVariable[pv], PRU_SERVOFREQ, updateHz, sensor, pinSensor, beta, r0, t0);
                    servoThread->registerModule(temperature);
                }
            }
            else if (!strcmp(type,"Switch"))
            {
                const char* comment = module["Comment"];
                printf("%s\n",comment);
    
                const char* pin = module["Pin"];
                const char* mode = module["Mode"];
                int pv = module["PV[i]"];
                float sp = module["SP"];
            
                printf("Make Switch (%s) at pin %s\n", mode, pin);
    
                if (!strcmp(mode,"On"))
                {
                    Module* SoftSwitch = new Switch(sp, *ptrProcessVariable[pv], pin, 1);
                    servoThread->registerModule(SoftSwitch);
                }
                else if (!strcmp(mode,"Off"))
                {
                    Module* SoftSwitch = new Switch(sp, *ptrProcessVariable[pv], pin, 0);
                    servoThread->registerModule(SoftSwitch);
                }
                else
                {
                    printf("Error - incorrectly defined Switch\n");
                }
            }
        }
        else if (!strcmp(thread,"On load"))
        {
            printf("\nOn load - run once module\n");

            if (!strcmp(type,"TMC stepper"))
            {
                printf("Make TMC");

                const char* driver = module["Driver"];
                printf("%s\n", driver);

                const char* comment = module["Comment"];
                printf("%s\n",comment);

                const char* RxPin = module["RX pin"];
                float RSense = module["RSense"];
                uint8_t address = module["Address"];
                uint16_t current = module["Current"];
                uint16_t microsteps = module["Microsteps"];
                const char* stealth = module["Stealth chop"];
                uint16_t stall = module["Stall sensitivity"];

                bool stealthchop;

                if (!strcmp(stealth, "on"))
                {
                    stealthchop = true;
                }
                else
                {
                    stealthchop = false;   
                }

                printf("%s\n", driver);

                if (!strcmp(driver, "2208"))
                {
                    // SW Serial pin, RSense, mA, microsteps, stealh
                    // TMC2208(std::string, float, uint8_t, uint16_t, uint16_t, bool);
                    Module* tmc = new TMC2208(RxPin, RSense, current, microsteps, stealthchop);
                
                    printf("\nStarting the COMMS thread\n");
                    commsThread->startThread();
                    commsThread->registerModule(tmc);
                    
                    tmc->configure();

                    printf("\nStopping the COMMS thread\n");
                    commsThread->stopThread();
                    commsThread->unregisterModule(tmc);
                    delete tmc;
                }
                else if (!strcmp(driver, "2209"))
                {
                    // SW Serial pin, RSense, addr, mA, microsteps, stealh, stall
                    // TMC2209(std::string, float, uint8_t, uint16_t, uint16_t, bool, uint16_t);
                    Module* tmc = new TMC2209(RxPin, RSense, address, current, microsteps, stealthchop, stall);
                
                    printf("\nStarting the COMMS thread\n");
                    commsThread->startThread();
                    commsThread->registerModule(tmc);
                    
                    tmc->configure();

                    printf("\nStopping the COMMS thread\n");
                    commsThread->stopThread();
                    commsThread->unregisterModule(tmc);
                    delete tmc;
                }
            }
        }
    }
}

void debugThread()
{
    commsThread->startThread();

    Module* debugOnC = new Debug("PE_4", 1);
    commsThread->registerModule(debugOnC);

    Module* debugOffC = new Debug("PE_4", 0);
    commsThread->registerModule(debugOffC); 
}

int main()
{    
    enum State currentState;
    enum State prevState;

    spiSlave.setStatus(false);
    spiSlave.setError(false);
    currentState = ST_SETUP;
    prevState = ST_RESET;

    printf("\nRemora PRU - Programmable Realtime Unit\n");

    motEnable = 1; // *** REMOVE THIS when motor enable module is implemented***

    watchdog.start(2000);

    while(1)
    {
      // the main loop does very little, keeping the Watchdog serviced and
      // resetting the rxData buffer if there is a loss of SPI commmunication
      // with LinuxCNC. Everything else is done via DMA and within the
      // two threads- Base and Servo threads that run the Modules.

    watchdog.kick();

    switch(currentState){
        case ST_SETUP:
            // do setup tasks
            if (currentState != prevState)
            {
                printf("\n## Entering SETUP state\n");
            }
            prevState = currentState;

            readJsonConfig();
            setup();
            loadModules();

            //debugThread();

            currentState = ST_START;
            break; 

        case ST_START:
            // do start tasks
            if (currentState != prevState)
            {
                printf("\n## Entering START state\n");
            }
            prevState = currentState;

            if (!threadsRunning)
            {
                // Start the threads
                printf("\nStarting the BASE thread\n");
                baseThread->startThread();
                
                printf("\nStarting the SERVO thread\n");
                servoThread->startThread();

                threadsRunning = true;

                // wait for threads to read IO before testing for PRUreset
                wait(1);
            }

            if (PRUreset)
            {
                // RPi outputs default is high until configured when LinuxCNC spiPRU component is started, PRUreset pin will be high
                // stay in start state until LinuxCNC is started
                currentState = ST_START;
            }
            else
            {
                currentState = ST_IDLE;
            }
            
            break;


        case ST_IDLE:
            // do something when idle
            if (currentState != prevState)
            {
                printf("\n## Entering IDLE state\n");
            }
            prevState = currentState;

            // check to see if there there has been SPI errors
            if (spiSlave.getError())
            {
                printf("SPI data error:\n");
                spiSlave.setError(false);
            }

            //wait for SPI data before changing to running state
            if (spiSlave.getStatus())
            {
                currentState = ST_RUNNING;
            }

            if (PRUreset) 
            {
                currentState = ST_WDRESET;
            }

            break;

        case ST_RUNNING:
            // do running tasks
            if (currentState != prevState)
            {
                printf("\n## Entering RUNNING state\n");
            }
            prevState = currentState;

            // check to see if there there has been SPI errors 
            if (spiSlave.getError())
            {
                printf("SPI data error:\n");
                spiSlave.setError(false);
            }
            
            if (spiSlave.getStatus())
            {
                // SPI data received by DMA
                resetCnt = 0;
                spiSlave.setStatus(false);
            }
            else
            {
                // no SPI data received by DMA
                resetCnt++;
            }

            if (resetCnt > SPI_ERR_MAX)
            {
                // reset threshold reached, reset the PRU
                printf("   SPI data error limit reached, resetting\n");
                resetCnt = 0;
                currentState = ST_RESET;
            }

            if (PRUreset) 
            {
                currentState = ST_WDRESET;
            }

            break;

        case ST_STOP:
            // do stop tasks
            if (currentState != prevState)
            {
                printf("\n## Entering STOP state\n");
            }
            prevState = currentState;


            currentState = ST_STOP;
            break;

        case ST_RESET:
            // do reset tasks
            if (currentState != prevState)
            {
                printf("\n## Entering RESET state\n");
            }
            prevState = currentState;

            // set all of the rxData buffer to 0
            // rxData.rxBuffer is volatile so need to do this the long way. memset cannot be used for volatile
            printf("   Resetting rxBuffer\n");
            {
                int n = sizeof(rxData.rxBuffer);
                while(n-- > 0)
                {
                    rxData.rxBuffer[n] = 0;
                }
            }

            currentState = ST_IDLE;
            break;

        case ST_WDRESET:
            // do a watch dog reset
            printf("\n## Entering WDRESET state\n");

            // force a watchdog reset by looping here
            while(1){}

            break;
      }

    wait(LOOP_TIME);
    }
}
