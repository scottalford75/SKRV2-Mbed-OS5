#ifndef SOFTWARESERIAL_H
#define SOFTWARESERIAL_H

#include "mbed.h"
#include <cstdint>
#include <string>
#include "../configuration.h"
#include "../pin/pin.h"


#define FORCE_BAUD_RATE 19600 //9600
#define IN_BUF_SIZE     64
#define TX_BITS         10     // 1 Startbit, 8 Databits, 1 Stopbit = 10 Bits/Frame
#define RX_BITS         8      // startbit and stopbit parsed internally (see ISR)
//#define OVERSAMPLE      3


class SoftwareSerial
{
    private:

    std::string     TXportAndPin;
    std::string     RXportAndPin;
    Pin*            txpin;
    Pin*            rxpin;

    //Ticker          ticker;
    
    unsigned char   inbuf[IN_BUF_SIZE];
    unsigned char   qin;
    unsigned char   qout;
    
    int32_t  baudRate;
 
    bool     activeTx;
    bool     activeRx;
    bool     halfDuplex;
    bool     outputPending;

    int32_t  rxTickCnt;
    int32_t  txTickCnt;
    int32_t  txBitCnt;
    int32_t  rxBitCnt;
    int32_t  txBuffer;
    int32_t  rxBuffer;

    public:

    SoftwareSerial(std::string, std::string);

    void begin(int);
    void setSpeed(int);
    void end(void);
    void setTX(void);
    void setRX(void);
    void setRXTX(bool);
    void send(void);
    void receive(void);
    void write(int);
    int16_t  read(void);
    bool listen(void);
    void tickerHandler(void);

    void enableTx(void);
    void enableRx(void);
    void idle() {__NOP();}
    

    int available();

    void flush_input_buffer();
    void printStr(char*);
};


#endif