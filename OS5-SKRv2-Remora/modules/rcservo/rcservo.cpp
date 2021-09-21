#include "rcservo.h"


/***********************************************************************
*                METHOD DEFINITIONS                                    *
************************************************************************/


RCServo::RCServo(volatile float &ptrPositionCmd, std::string pin, int32_t threadFreq, int32_t slowUpdateFreq) :
	Module(threadFreq, slowUpdateFreq),
	ptrPositionCmd(&ptrPositionCmd),
	pin(pin),
	threadFreq(threadFreq)
{
	cout << "Creating RC servo at pin " << this->pin << endl;

	this->servoPin = new Pin(this->pin, OUTPUT);			// create Pin

	this->T_ms = 20; 	// 50hz
	this->T_compare = this->T_ms * this->threadFreq / 1000;
	this->pinState = false;
	this->counter = 0;
	this->positionCommand = 0;
	
	this->t_compare = (this->threadFreq / 1000)*(1 + (int)this->positionCommand / 180);
}

void RCServo::update()
{
	counter++;

	if (counter == this->t_compare)
	{
		pinState = false;		// falling edge of pulse
	}
	else if (counter == this->T_compare)
	{
		pinState = true; 		// rising edge of pulse
		counter = 0;
	}

	this->servoPin->set(pinState);
}

void RCServo::slowUpdate()
{
	// the slowUpate is used to update the position set-point

	this->positionCommand = *(this->ptrPositionCmd); 
	int t = this->threadFreq*(180 + (int)this->positionCommand)/(1000*180);
	this->t_compare = (int)t;
}