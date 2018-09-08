#ifndef RECEIVER_H
#define RECEIVER_H

//Check for risk of overflow when timing ?

class Receiver{
	public:
	Receiver(int pinRoll, int pinPitch, int pinThrottle, int pinYaw);
	//STOP Interrupts ?

	int getRoll(){return this.timeRoll;}
	int getPitch(){return this.timePitch;}
	int getYaw(){return this.timeYaw;}
	int getThrottle(){return this.timeThrottle;}

	void stop();
	void start();

	private:
	volatile int timeRoll, timePitch, timeThrottle, timeYaw;
	volatile int tmpTimeRoll, tmpTimePitch, tmpTimeThrottle, tmpTimeYaw;

	const int pinRoll, pinPitch, pinThrottle, pinYaw;
	void timingRoll();
	void timingPitch();
	void timingYaw();
	void timingThrottle();

}

#endif RECEIVER_H
