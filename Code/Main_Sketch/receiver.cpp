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

int Receiver::getRoll(){
	return this.timeRoll;
}

int Receiver::getPitch(){
	return this.timePitch;
}

int Receiver::getThrottle(){
	return this.timeThrottle;
}

int Receiver::getYaw(){
	return this.timeYaw;
}