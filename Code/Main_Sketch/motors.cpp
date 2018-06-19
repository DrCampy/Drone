#include "motors.h"

Motor::Motor(int pin, int microsLow, int microsHigh, int microsRunning){
	this.microsLow = microsLow; //Useless ?
	this.microsHigh = microsHigh;
	this.microsRunning = microsRunning;
	this.ESC.attach(pin, microsLow, microsHigh);
	dearm();

}

void Motor::dearm(){
	this.internalState = DISARMED;
	ESC.write(0);
}

void Motor::arm(){
	if(this.internalState == DISARMED){
		this.internalState = ARMED;
		this.ESC.write(microsRunning);
	}
}

void Motor::write(int micros){
	if(this.internalState == ARMED){
		if(micros < this.microsRunning){
			this.ESC.write(microsRunning);
		}else if(micros > this.microsHigh){
			this.ESC.write(microsHigh);
		}else{
			this.ESC.write(micros);
		}
	}
}

	