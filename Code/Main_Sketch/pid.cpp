#include "pid.h"

PID::PID(unsigned long tSample, float kPRoll, float kIRoll, float kDRoll, float kPPitch, float kIPitch, float kDPitch, float kPYaw, float kIYaw, float kDYaw){
	this.tSample = tSample;

	this.kPRoll = kPRoll;
	this.kIRoll = kIRoll;
	this.kDRoll = kDRoll;

	this.kPPitch = kPPitch;
	this.kIPitch = kIPitch;
	this.kDPitch = kDPitch;

	this.kPYaw = kPYaw;
	this.kIYaw = kIYaw;
	this.kDYaw = kDYaw;

	this.prevErrorRoll = 0;
	this.prevErrorPitch = 0;
	this.prevErrorYaw = 0;

	this.IRoll = 0;
	this.IPitch = 0;
	this.IYaw = 0;

	this.rollSum = 0;
	this.pitchSum = 0;
	this.yawSum = 0;
}

void PID::update(float actRoll, float actPitch, float actYaw, float reqRoll, float reqPitch, float reqYaw){

	//computes errors valus
	float rollError = reqRoll - actRoll;
	float pitchError = reqPitch - actPitch;
	float yawError = reqYaw - actYaw;

	//Proportional part
	float PRoll = kPRoll*rollError;
	float PPitch = kPPitch*pitchError;
	float PYaw = kPYaw*yawError;

	//Integral part
	IRoll = IRoll + kIRoll*rollError*tSample;
	IPitch = IPitch + kIPitch*pitchError*tSample;
	IYaw = IYaw + kIYaw*yawError*tSample;

	//Derivative part
	float DRoll = kDRoll*(prevErrorRoll - rollError)/tSample;
	float DPitch = kDPitch*(prevErrorPitch - pitchError)/tSample;
	float DYaw = kDYaw*(prevErrorYaw-yawError)/tSample;

	//Stores the error of this iteration
	prevErrorRoll = rollError;
	prevErrorPitch = pitchError;
	prevErrorYaw = yawError;

	//constrain the integral part to avoid excessive values
	IRoll = constrain(IRoll, -100, 100);
	IPitch = constrain(IPitch, -100, 100);
	IYaw = constrain(IYaw, -100, 100);

	//Computes sums
	rollSum = PRoll + IRoll + DRoll;
	pitchSum = PPitch + IPitch + DPitch;
	yawSum = PYaw + IYaw + DYaw;
}
