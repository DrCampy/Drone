//setegvsgvsjhvgshjgv
#include <Wire.h> // communication avec les différents capteurs
#include <Servo.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#define USE_COMPASS

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_L3GD20_U.h>

#ifdef USE_COMPASS
#include <Adafruit_HMC5883_U.h>
#endif


//connexion au recepteur
#define ROLL_PIN 8 //CH1
#define PITCH_PIN 9 //CH2
#define THROTTLE_PIN 11 //CH3
#define YAW_PIN 10 //CH4

#define N_ROLL 0
#define N_PITCH 1
#define N_THROTTLE 2
#define N_YAW 3

#define PIN_ESC_AVD A0
#define PIN_ESC_AVG A3
#define PIN_ESC_ARD A1
#define PIN_ESC_ARG A2
#define PIN_BUZZER 4

#define MICROS_LOW 1050
#define MICROS_HIGH 1900
#define MICROS_RUNNING MICROS_LOW + 20


void setup();
void loop();

void receiver_to_value();
void read_gy80();
void calculate_power();
void send_signal();
void pid();
float map_float(double x, double in_min, double in_max, double out_min, double out_max);
void calibrate_gyroscope(const int);
#ifdef USE_COMPASS
void calibrate_compass(const float);
#endif
void buzzer(int number);

/*to be implemented
  zero_when_zero()
  fonction pour armer les moteurs (throttle = 0 et rot autour axe z = -180)
*/

typedef struct {
  float x = 0;
  float y = 0;
  float z = 0;
} coordinates;

typedef struct {
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
} angles;

typedef struct {
  struct {
    coordinates scale;
    coordinates offset;
  } compass;
  struct {
    angles drift;
  } gyro;

  struct {
    coordinates offset;
  } accel;
} calibration;

calibration cal; //calibration datas of the different captors
coordinates accel; //raw values of the accel //float accel_x = 0, accel_y = 0, accel_z = 0; //valeurs corrigées de l'accéléromètre (read_gy80)
coordinates gyro; //raw values of the gyro float X_vel = 0, Y_vel = 0, Z_vel = 0; //valeurs corrigées du gyroscope

int debug = 0;

unsigned long int last_sample = 0; //Pour une bonne frequelce de poll sur le magnetometre

const float alpha = 0.9; //low_pass filter accelerometre

float roll_a = 0, pitch_a = 0; //angles donnés par l'accelerometre

float roll_g = 0, pitch_g = 0, yaw_g = 0;//angles (deg) calculés aptd gyroscope

float yaw_angle = 0, roll_angle = 0, pitch_angle = 0; //read_gy80 output

//used for integration
float dt_gy80 = 0, dt_pid = 0;
unsigned long int timer_pid = 0,  timer_gy80 = 0;

int value_throttle = 0; //valeur des gaz (envoyée par la radio)
float value_roll = 0, value_pitch = 0, value_yaw = 0; //valeurs envoyées par la radio

/****************************************************************************************************/
/*                                     Variables du PID                                             */
/****************************************************************************************************/


float X_sum = 0, Y_sum = 0, Z_sum = 0;//sommes des différents elements du PID
float previous_roll_error = 0;
float previous_pitch_error = 0;
float previous_yaw_error = 0;

#ifdef USE_COMPASS
float var_compass = 0; //direction pointee par la boussole
#endif

//Valeurs des gains pour le PID
float k_P_xy = 2, k_I_xy = 1, k_D_xy = 1;
float k_P_z = 2, k_I_z = 1, k_D_z = 1;

float P_x = 0, P_y = 0, P_z = 0;
float I_x = 0, I_y = 0, I_z = 0;
float D_x = 0, D_y = 0, D_z = 0;

int indice = 0;

unsigned short int power_AVG = 0, power_AVD = 0, power_ARD = 0, power_ARG = 0;

sensors_event_t val_accel, val_gyro, val_compass;

Adafruit_ADXL345_Unified accelerometre = Adafruit_ADXL345_Unified(1);//12345 remplace par 00345
Adafruit_L3GD20_Unified gyroscope = Adafruit_L3GD20_Unified(2);

#ifdef USE_COMPASS
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(3);
#endif

void setup() {

  Serial.begin(9600);
  while (!Serial) ;

  //On vérifie que les capteurs on été correctement connectée.
  if (!gyroscope.begin(GYRO_RANGE_250DPS)) {
    while(true){}
  }
  if (!accelerometre.begin()) {
    while(true){}
  }

#ifdef USE_COMPASS
  if (!compass.begin()) {
    while(true){}
  }
#endif

  //définition des echelles des capteurs
  buzzer(1);
  calibrate_gyroscope(300);
  buzzer(2);
  
#ifdef USE_COMPASS
  compass.setMagGain(HMC5883_MAGGAIN_1_9);
  calibrate_compass(60);
  buzzer(3);
#endif

  for (indice = 0; indice < 60; indice++) {
    delay(10);
    gyroscope.getEvent(&val_gyro);
  }
  
  //waiting for user to arm the drone
  unsigned int counter = 0;
  while(counter < 1700){
    counter ++;
    gyroscope.getEvent(&val_gyro);
    if(time_ch[N_YAW] > 1150 || time_ch[N_THROTTLE] > 1090){
      counter = 0;  
    }
  }

  
  buzzer(3);
  roll_angle = 0;
  pitch_angle = 0;
  yaw_angle = 0;

  //NEW CODE
  Motor MotorAVG = new Motor(PIN_ESC_AVG, MICROS_LOW, MICROS_HIGH);
  Motor MotorAVD = new Motor(PIN_ESC_AVD, MICROS_LOW, MICROS_HIGH);
  Motor MotorARD = new Motor(PIN_ESC_ARD, MICROS_LOW, MICROS_HIGH);
  Motor MotorARG = new Motor(PIN_ESC_ARG, MICROS_LOW, MICROS_HIGH);

  Receiver receiver(ROLL_PIN, PITCH_PIN, THROTTLE_PIN, YAW_PIN);

}


void loop() {

  if (debug == 0) {
    roll_angle = 0;
    pitch_angle = 0;
    yaw_angle = 0;
    roll_g = 0;
    pitch_g = 0;
    yaw_g = 0;
    debug++;
    for (indice = 0; indice < 60; indice++) {
      delay(10);
      gyroscope.getEvent(&val_gyro);
    } //flush old datas of the gyroscope
  }

  receiver_to_value();
  
  if (value_throttle < 1100) {
    stand_still();
  } else {
  gyroscope.getEvent(&val_gyro);
  read_gy80();
  pid();
  calculate_power();
  }
  
  send_signal();
/*
  if (indice == 200) {

    Serial.println("Angles (gyro): ");
    Serial.print("Roll_angle:  "); Serial.print(roll_angle); Serial.print("  ");
    Serial.print("Pitch_angle: "); Serial.print(pitch_angle); Serial.print("  ");
    Serial.print("yaw_angle:   "); Serial.print(yaw_angle); Serial.print("  "); Serial.println("deg");

    Serial.println("Angles (accelerometre): ");
    Serial.print("Roll_a:  "); Serial.print(roll_a); Serial.print("  ");
    Serial.print("Pitch_a: "); Serial.print(pitch_a); Serial.print("  "); Serial.println("deg");

    Serial.print("Value throttle");Serial.println(value_throttle);
    Serial.print("Value Roll");Serial.println(value_roll);
    Serial.print("Value Pitch");Serial.println(value_pitch);
    Serial.print("Value Yaw");Serial.println(value_yaw);

    Serial.print("X_Sum: ");Serial.println(X_sum);
    Serial.print("Y_Sum: ");Serial.println(Y_sum);
    Serial.print("Z_Sum: ");Serial.println(Z_sum);
    
    /*
      Serial.print("Micros sur Channel 1: ");
      Serial.println(time_ch[N_ROLL]);
      Serial.print("Micros sur Channel 2: ");
      Serial.println(time_ch[N_PITCH]);
      Serial.print("Micros sur Channel 3: ");
      Serial.println(time_ch[N_THROTTLE]);
      Serial.print("Micros sur Channel 4: ");
      Serial.println(time_ch[N_YAW]);
    */
/*
    Serial.print("Signal to AVG: ");
    Serial.println(power_AVG);
    Serial.print("Signal to AVD: ");
    Serial.println(power_AVD);
    Serial.print("Signal to ARD: ");
    Serial.println(power_ARD);
    Serial.print("Signal to ARG: ");
    Serial.println(power_ARG);

#ifdef USE_COMPASS
    Serial.print("Compass_heading: "); Serial.println(var_compass);
#endif


    Serial.println("-----------------------------------------------------------------------------");
  }*/
  //indice = (indice + 1) % 201;
}


void stand_still() {
  power_AVG = power_AVD = power_ARD = power_ARG = 1000;
  I_x = I_y = I_z = 0;
}


void read_gy80() {

  //double  atan2 (double __y, double __x)  // arc tangent of y/x

  float k = 0.95;

  accelerometre.getEvent(&val_accel);
  val_accel.acceleration.x -= cal.accel.offset.x;
  val_accel.acceleration.y -= cal.accel.offset.y;
  val_accel.acceleration.z -= cal.accel.offset.z;

  accel.x = val_accel.acceleration.x * alpha + ( accel.x * (1.0 - alpha));
  accel.y = val_accel.acceleration.y * alpha + ( accel.y * (1.0 - alpha));
  accel.z = val_accel.acceleration.z * alpha + ( accel.z * (1.0 - alpha));

  /******************************************************************************/
  /* Ce bloc sert à transformer les données de l'accéléromètre (en g par axe)   */
  /* en un angle par inclinaison (pitch/roll) decrivant sa position!            */
  /******************************************************************************/
  roll_a = ((atan2(-accel.y, accel.z) * 180.0) / M_PI); //accelero
  pitch_a = ((atan2(accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)) * 180.0) / M_PI); //accelero




#ifdef USE_COMPASS
  if ((micros() - last_sample) > (1 / 30) / 1000000) {
    last_sample = micros();
    //scale * (data-offset)
    compass.getEvent(&val_compass);
    val_compass.magnetic.x = cal.compass.scale.x * (val_compass.magnetic.x - cal.compass.offset.x);
    val_compass.magnetic.y = cal.compass.scale.y * (val_compass.magnetic.y - cal.compass.offset.y);
    val_compass.magnetic.z = cal.compass.scale.z * (val_compass.magnetic.z - cal.compass.offset.z);

    float xh = val_compass.magnetic.x * cos(pitch_angle) + val_compass.magnetic.y * sin(pitch_angle) * sin(roll_angle) - val_compass.magnetic.z * cos(roll_angle) * sin(pitch_angle);
    float yh = val_compass.magnetic.y * cos(roll_angle) + val_compass.magnetic.z * sin(roll_angle);

    if (yh > 0.1) {
      var_compass = 90 - atan2((float)xh, (float)yh) * (180 / PI); // angle in degrees
    } else if (yh < -0.1) {
      var_compass = 270 - atan2((float)xh, (float)yh) * (180 / PI);
    } else {
      if (xh < 0) {
        var_compass = 180;
      } else if (xh > 0) {
        var_compass = 0;
      }
    }

    var_compass = fmod(var_compass, 360);
  }
#endif

  gyroscope.getEvent(&val_gyro);
  dt_gy80 = ((float)(micros() - timer_gy80) / 1000000);
  val_gyro.gyro.x -= cal.gyro.drift.roll;
  val_gyro.gyro.y -= cal.gyro.drift.pitch;
  val_gyro.gyro.z -= cal.gyro.drift.yaw;

  //Complementary filter
  roll_angle = k * (roll_angle + val_gyro.gyro.x * dt_gy80) + (1 - k) * roll_a;
  pitch_angle = k * (pitch_angle + val_gyro.gyro.y * dt_gy80) + (1 - k) * pitch_a;
  yaw_angle = yaw_angle + val_gyro.gyro.z * dt_gy80;
  timer_gy80 = micros();
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
    value_roll = map_float(tmp, 1050, 1460, -45, 0);
  else if (tmp > 1490)
    value_roll = map_float(tmp, 1490, 1900, 0, 45);
  else
    value_roll = 0; //deg

  tmp = time_ch[N_PITCH];
  if (tmp < 1460)
    value_pitch = map_float(tmp, 1050, 1460, -45, 0);
  else if (tmp > 1490)
    value_pitch = map_float(tmp, 1490, 1900, 0, 45);
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


void pid() { // VIDE
  float roll_error = (value_roll - roll_angle);
  float pitch_error = (value_pitch - pitch_angle );
  float yaw_error = (value_yaw - yaw_angle  );

  dt_pid = ((float)(micros() - timer_pid) / 1000000);

  //Serial.print("Angles: (roll yaw pitch): "); Serial.print(roll_angle);Serial.print(yaw_angle);Serial.println(pitch_angle);
  //Serial.print("Values (throttle roll yaw pitch): "); Serial.print(value_throttle);Serial.print(value_roll); Serial.print(value_yaw); Serial.println(value_pitch);

  P_x = roll_error * k_P_xy; //roll value_roll; == rad_tilt LR
  I_x = I_x + (roll_error * dt_pid * k_I_xy);
  D_x = (roll_error - previous_roll_error) * (k_D_xy / dt_pid);

  P_y = pitch_error * k_P_xy; //pitch value_pitch; == rad_tilt TB
  I_y = I_y + (pitch_error * dt_pid * k_I_xy);
  D_y = (pitch_error - previous_pitch_error) * (k_D_xy / dt_pid);

  P_z = yaw_error * k_P_z;
  I_z = I_z + ( yaw_error * dt_pid * k_I_z);
  D_z = (yaw_error - previous_yaw_error) * (k_D_z / dt_pid);

  timer_pid = micros();

  if(I_x > 35)
    I_x = 35;
  if(I_x <-35)
    I_x = -35;
    
  if(I_y > 35)
    I_y = 35;
  if(I_y <-35)
    I_y = -35;
    
  if(I_z > 35)
    I_z = 35;
  if(I_z <-35)
    I_z = -35;  
  previous_roll_error = roll_error;
  previous_pitch_error = pitch_error;
  previous_yaw_error = yaw_error;

  X_sum = P_x + I_x + D_x;
  Y_sum = P_y + I_y + D_y;
  Z_sum = P_z + I_z + D_z;
}


void send_signal() { // VIDE
  ESC_AVG.writeMicroseconds(power_AVG);
  ESC_AVD.writeMicroseconds(power_AVD);
  ESC_ARD.writeMicroseconds(power_ARD);
  ESC_ARG.writeMicroseconds(power_ARG);
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


#ifdef USE_COMPASS
void calibrate_compass(const float time_seconds) {
  unsigned long int time_micros = (unsigned long int) time_seconds * 1000000;
  float max_x = 0, max_y = 0, max_z = 0;
  float min_x = 0, min_y = 0, min_z = 0;
  float field = 0;
  unsigned long int debut = 0;

  compass.getEvent(&val_compass);
  min_x = max_x = val_compass.magnetic.x;
  min_y = max_y = val_compass.magnetic.y;
  min_z = max_z = val_compass.magnetic.z;

  debut = micros();
  while ( micros() - debut < time_micros) {
    compass.getEvent(&val_compass);

    min_x = min(min_x, val_compass.magnetic.x);
    max_x = max(max_x, val_compass.magnetic.x);

    min_y = min(min_y, val_compass.magnetic.y);
    max_y = max(max_y, val_compass.magnetic.y);

    min_z = min(min_z, val_compass.magnetic.z);
    max_z = max(max_z, val_compass.magnetic.z);
  }

  cal.compass.offset.x = (min_x + max_x) / 2;
  cal.compass.offset.y = (min_y + max_y) / 2;
  cal.compass.offset.z = (min_z + max_z) / 2;

  field = ( (max_x - min_x) + (max_y - min_y) + (max_z - min_z) ) / 3;
  //field = ( (max_x) + (max_y) + (max_z) ) / 3;

  cal.compass.scale.x = field / (max_x - min_x);
  cal.compass.scale.y = field / (max_y - min_y);
  cal.compass.scale.z = field / (max_z - min_z);

  /*cal.compass.scale.x = field / (max_x);
    cal.compass.scale.y = field / (max_y);
    cal.compass.scale.z = field / (max_z);*/
}
#endif


void buzzer(int number){
  
  for(int i = 0; i < number; i++){
    digitalWrite(PIN_BUZZER, HIGH);
    
    for(int i = 0; i < 25000; i++){
      
    }
    digitalWrite(PIN_BUZZER, LOW);
    for(int i = 0; i < 15000; i++){
    
    }
  }
}

