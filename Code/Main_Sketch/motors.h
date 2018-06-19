#ifndef MOTORS_H
#define MOTORS_H

#include <Servo.h>

class Motor {
  
  public:
  Motor(int pin, int microsLow, int microshigh, int microsRunning);
  void dearm();
  void arm();
  void write(int micros);
  
  private:
  enum State{DISARMED, RUNNING};
  State internalState;
  const int microsLow, microshigh, microsRunning;
  Servo ESC;
  
};

#endif
