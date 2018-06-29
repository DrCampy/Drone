#ifndef PID_H
#define PID_H

class PID{
	private:
	const float kPRoll, kIRoll, kDRoll;
	const float kPPitch, kIPitch, kDPitch;
	const float kPYaw, kIYaw, kDYaw;
	
	const unsigned long tSample; //in microseconds
	
	float prevErrorRoll, prevErrorPitch, prevErrorYaw;
	float IRoll, IPitch, IYaw;
	
	float rollSum, pitchSum, yawSum;
	
	public:
	PID(unsigned long tSample, float kPRoll, float kIRoll, float kDRoll, float kPPitch, float kIPitch, float kDPitch, float kPYaw, float kIYaw, float kDYaw);
	void update(float actRoll, float actPitch, float actYaw, float reqRoll, float reqPitch, float reqYaw);
	
	void getRollVal(){return rollSum;}
	void getPitchVal(){return pitchSum;}
	void getYawVal(){return yawSum;}
	
	
}

#endif