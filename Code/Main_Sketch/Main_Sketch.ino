//setegvsgvsjhvgshjgv
#include <Wire.h> // communication avec les différents capteurs
#include <Servo.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_HMC5883_U.h>


//connexion au recepteur
#define ROLL_PIN 8 //CH1
#define PITCH_PIN 9 //CH2
#define THROTTLE_PIN 11 //CH3
#define YAW_PIN 10 //CH4

#define PIN_ESC_AVD A0
#define PIN_ESC_AVG A3
#define PIN_ESC_ARD A1
#define PIN_ESC_ARG A2
#define PIN_BUZZER 4

#define MICROS_LOW 1050
#define MICROS_HIGH 1900
#define MICROS_RUNNING MICROS_LOW + 20

#define ACCEL_I2C_ADDR 1
#define MAG_I2C_ADDR 2
#define GYRO_I2C_ADDR 3

#define PITCH_ANGLE_LIMIT 45 //Pitch will vary between -PITCH_ANGLE_LIMIT and PITCH_ANGLE_LIMIT
#define ROLL_ANGLE_LIMIT 45 //Roll will vary between -ROLL_ANGLE_LIMIT and ROLL_ANGLE_LIMITS
#define K_THROTTLE 50 //Multiply throttle by this factor (limits total power to k*100%)

//Variables
Adafruit_ADXL345_Unified accel(ACCEL_I2C_ADDR);
Adafruit_HMC5883_Unified mag(MAG_I2C_ADDR);
Adafruit_L3GD20_Unified gyro(GYRO_I2C_ADDR);

//Sensors values
float mx = 0.0F, my = 0.0F, mz = 0.0F; //magnetometer
float ax = 0.0F, ay = 0.0F, az = 0.0F; //Accelerometer
float gx = 0.0F, gy = 0.0F, gz = 0.0F; //Gyroscopic

//RC values
unsigned int tRoll = 0, tPitch = 0, tYaw = 0, tThrottle = 0;

//desired values in angles
unsigned int aRoll = 0, aPitch = 0, aYaw = 0;

//times to send to each motor
unsigned int tFL = 0, tFR = 0, tRR = 0, tRL = 0;

// Offsets applied to raw x/y/z values
float mag_offsets[3] = { -2.20F, -5.53F, -26.34F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = {{ 0.934, 0.005, 0.013 }, { 0.005, 0.948, 0.012 }, { 0.013, 0.012, 1.129 }};

//float mag_field_strength = 48.41F;

Madgwick AHRS_filter;
unsigned long tSample = 1000000/50; //50Hz sampling rate
unsigned long microsPrevious = 0;

void setup() {

  Serial.begin(9600);
  while (!Serial) ;

  //On vérifie que les capteurs on été correctement connectée.
  gyro.setRange(
  if (!gyro.begin(GYRO_RANGE_500DPS)) {
    while(true){}
  }
  if (!accel.begin(ADXL345_RANGE_2G)) {
    while(true){}
  }
  if (!mags.begin(HMC5883_MAGGAIN_1_3)) {
    while(true){}
  }

  for (indice = 0; indice < 60; indice++) {
    delay(10);
    gyroscope.getEvent(&val_gyro);
  }

  //waiting for user to arm the drone TODO

  //NEW CODE
  Motor MotorAVG = new Motor(PIN_ESC_AVG, MICROS_LOW, MICROS_HIGH);
  Motor MotorAVD = new Motor(PIN_ESC_AVD, MICROS_LOW, MICROS_HIGH);
  Motor MotorARD = new Motor(PIN_ESC_ARD, MICROS_LOW, MICROS_HIGH);
  Motor MotorARG = new Motor(PIN_ESC_ARG, MICROS_LOW, MICROS_HIGH);

  Receiver receiver(ROLL_PIN, PITCH_PIN, THROTTLE_PIN, YAW_PIN);

}


void loop() {

	sensors_event_t gyro_event;
	sensors_event_t accel_event;
	sensors_event_t mag_event;

  PID pid = PID(tSample, );
  //PID(unsigned long tSample, float kPRoll, float kIRoll, float kDRoll, float kPPitch, float kIPitch, float kDPitch, float kPYaw, float kIYaw, float kDYaw);


  //Iterate every tSample seconds
	if(micros() - microsPrevious >= tSample){
		gyro.getEvent(&gyro_event);
		accel.getEvent(&accel_event);
		mag.getEvent(&mag_event);

		// Apply mag offset compensation (base values in uTesla)
		float x = mag_event.magnetic.x - mag_offsets[0];
		float y = mag_event.magnetic.y - mag_offsets[1];
		float z = mag_event.magnetic.z - mag_offsets[2];

		// Apply mag soft iron error compensation
		float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
		float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
		float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

		// The filter library expects gyro data in degrees/s, but adafruit sensor
		// uses rad/s so we need to convert them first (or adapt the filter lib
		// where they are being converted)
		float gx = gyro_event.gyro.x * 57.2958F;
		float gy = gyro_event.gyro.y * 57.2958F;
		float gz = gyro_event.gyro.z * 57.2958F;

		AHRS_filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    //Actual values of the position of the drone.
		roll = AHRS_filter.getRoll();
		pitch = AHRS_filter.getPitch();
		yaw = AHRS_filter.getYaw();

    //Asked values
    tRoll = receiver.getRoll();
    tPitch = receiver.getPitch();
    tYaw = receiver.getYaw(); //This is a rotational speed
    tThrottle = receiver.getThrottle();

    if(tThrottle < 1100){
      stand_still();
      tFL = MICROS_RUNNING;
      tFR = MICROS_RUNNING;
      tRR = MICROS_RUNNING;
      tRL = MICROS_RUNNING;
      pid.zeroI();
    }else{
      if (tRoll < 1460)
        aRoll = map_float(tRoll, MICROS_LOW, 1460, -25, 0);
      else if (tmp > 1490)
        aRoll = map_float(tRoll, 1490, MICROS_HIGH, 0, 25);
      else
        aRoll = 0; //deg

      if (tPitch < 1460)
        aPitch = map_float(tPitch, MICROS_LOW, 1460, -25, 0);
      else if (tPitch > 1490)
        aPitch = map_float(tPitch, 1490, MICROS_HIGH, 0, 25);
      else
        aPitch = 0; //deg


      }


      if (value_throttle < 1100) {
      } else {
  gyroscope.getEvent(&val_gyro);
  read_gy80();
  pid();
  calculate_power();
  }

  send_signal();
}


void stand_still() {
  power_AVG = power_AVD = power_ARD = power_ARG = 1000;
  I_x = I_y = I_z = 0;
}

void receiver_to_value() {
  //short unsigned int tmp = 0;
  int tmp = 0;

  tmp = time_ch[N_THROTTLE];
  if (tmp > 1100)
    value_throttle = tmp;
  else
    value_throttle = 900;

  tmp = time_ch[N_ROLL];
  if (tmp < 1460)
    value_roll = map_float(tmp, 1050, 1460, -25, 0);
  else if (tmp > 1490)
    value_roll = map_float(tmp, 1490, 1900, 0, 25);
  else
    value_roll = 0; //deg

  tmp = time_ch[N_PITCH];
  if (tmp < 1460)
    value_pitch = map_float(tmp, 1050, 1460, -25, 0);
  else if (tmp > 1490)
    value_pitch = map_float(tmp, 1490, 1900, 0, 25);
  else
    value_pitch = 0; //deg

  tmp = time_ch[N_YAW];
  if (tmp < 1460)
    value_yaw = map_float(tmp, 1050, 1460, -180, 0);
  else if (tmp > 1490)
    value_yaw = map_float(tmp, 1490, 1900, 0, 180);
  else
    value_yaw = 0; //deg/s
}


void calculate_power() { //VIDE
  power_AVG = (int) (value_throttle - Z_sum - Y_sum + X_sum);
  power_ARD = (int) (value_throttle - Z_sum + Y_sum - X_sum);
  power_AVD = (int) (value_throttle + Z_sum - Y_sum - X_sum);
  power_ARG = (int) (value_throttle + Z_sum + Y_sum + X_sum);

  power_AVG = constrain(power_AVG, 1100, 1900);
  power_AVD = constrain(power_AVD, 1100, 1900);
  power_ARD = constrain(power_ARD, 1100, 1900);
  power_ARG = constrain(power_ARG, 1100, 1900);
}

float map_float(double x, double in_min, double in_max, double out_min, double out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void calibrate_gyroscope(const int nbr_loop) {
  int indice = 0;
  float gyro_sum_roll = 0, gyro_sum_pitch = 0, gyro_sum_yaw = 0;

  for (indice = 0; indice < 35; indice++) {
    gyroscope.getEvent(&val_gyro);
    delay(20);
  }
  for (indice = 0; indice < nbr_loop; indice++) {
    gyroscope.getEvent(&val_gyro);
    gyro_sum_roll += val_gyro.gyro.x;
    gyro_sum_pitch += val_gyro.gyro.y;
    gyro_sum_yaw += val_gyro.gyro.z;
    delay(15);
  }

  cal.gyro.drift.roll = (float) gyro_sum_roll / nbr_loop;
  cal.gyro.drift.pitch = (float) gyro_sum_pitch / nbr_loop;
  cal.gyro.drift.yaw = (float) gyro_sum_yaw / nbr_loop;
}
