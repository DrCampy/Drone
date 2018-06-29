#include "receiver.h"
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

Receiver::Receiver(	int pinRoll, int pinPitch, int pinThrottle, int pinYaw){
	this.timeRoll = 0;
	this.timePitch = 0;
	this.timeThrottle = 0;
	this.timeYaw = 0;
	
	this.tmpTimeRoll = 0;
	this.tmpTimePitch = 0;
	this.tmpTimeThrottle = 0;
	this.tmpTimeYaw = 0;
	
	this.pinRoll = pinRoll;
	this.pinPitch = pinPitch;
	this.pinThrottle = pinThrottle;
	this.pinYaw = pinYaw;

	
	pinMode(pinRoll, INPUT);
	pinMode(pinPitch, INPUT);
	pinMode(pinThrottle, INPUT);
	pinMode(pinYaw, INPUT);
  
	attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(pinRoll), timingRoll, CHANGE);
	attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(pinPitch), timingPitch, CHANGE);
	attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(pinThrottle), timingThrottle, CHANGE);
	attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(pinYaw), timingYaw, CHANGE);
}

void Receiver::timingRoll() {
  if (digitalRead(pinRoll))
    tmpTimeRoll = micros();
  else
    timeRoll = micros() - tmpTimeRoll;
}


void Receiver::timingPitch() {
  if (digitalRead(pinPitch))
    tmpTimePitch = micros();
  else
    timepitch = micros() - tmpTimePitch;
}


void Receiver::timingThrottle() {
  if (digitalRead(pinThrottle))
    tmpTimeThrottle = micros();
  else
    timeThrottle = micros() - tmpTimeThrottle;
}


void Receiver::timingYaw() {
  if (digitalRead(pinYaw))
    tmpTimeYaw = micros();
  else
    timeYaw = micros() - tmpTimeYaw;
}

void Receiver::stop(){
	disablePCINT(digitalPinToPinChangeInterrupt(pinRoll));
	disablePCINT(digitalPinToPinChangeInterrupt(pinPitch));
	disablePCINT(digitalPinToPinChangeInterrupt(pinThrottle));
	disablePCINT(digitalPinToPinChangeInterrupt(pinYaw));
}

void Receiver::start(){
	enablePCINT(digitalPinToPinChangeInterrupt(pinRoll));
	enablePCINT(digitalPinToPinChangeInterrupt(pinPitch));
	enablePCINT(digitalPinToPinChangeInterrupt(pinThrottle));
	enablePCINT(digitalPinToPinChangeInterrupt(pinYaw));
	
	unsigned long time = millis();
	
	while(millis()-time < 3); //wait for 2 about 2 pulses so that our readings will be correct
}
