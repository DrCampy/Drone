#ifndef RECEIVER_H
#define RECEIVER_H

//Check for risk of overflow when timing ?

class Receiver{
	public:
	Receiver(int pinRoll, int pinPitch, int pinThrottle, int pinYaw);
	//STOP Interrupts ?
	
	int getRoll();
	int getPitch();
	int getThrottle();
	int getYaw();
	
	private:
	volatile int timeRoll, timePitch, timeThrottle, timeYaw;
	volatile int tmpTimeRoll, tmpTimePitch, tmpTimeThrottle, tmpTimeYaw;

	const int pinRoll, pinPitch, pinThrottle, pinYaw;
	void timingRoll();
	void timingPitch();
	void timingThrottle();
	void timingYaw();
	
}

#endif RECEIVER_H