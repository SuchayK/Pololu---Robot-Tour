#include <Wire.h>
#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>

#include <Pololu3piPlus32U4IMU.h>

#include <Hashtable.h> 

using namespace Pololu3piPlus32U4;

OLED display;

Encoders encoders;
Buzzer buzzer;
Motors motors;
IMU imu;

const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.05; //10.0531

volatile int leftC = 0;
volatile int rightC = 0;
long prevT = 0;
float leprev = 0;
float leintegral = 0;
float reprev = 0;
float reintegral = 0;

void movePID(int target) {
  float kp = 0.1;
  float ki = 0.05; 
  float kd = 0.01; 
  int maxSpeed = 120;
  float iLimit = 1000;

  leftC = 0;
  rightC = 0;
  prevT = micros();

  while (leftC < target && rightC < target) {
    leftC += encoders.getCountsAndResetLeft();
    rightC += encoders.getCountsAndResetRight();

    long currT = micros();
    float deltaT = ((float) (currT - prevT)) / 1.0e6;
    prevT = currT;

    int le = target - leftC;
    int re = target - rightC;

    float ldedt = (le - leprev) / deltaT;
    float rdedt = (re - reprev) / deltaT;

    if (abs(le) < iLimit) {
      leintegral += le * deltaT;
    }
    if (abs(re) < iLimit) {
      reintegral += re * deltaT;
    }

    int lpwr = kp * le + ki * leintegral + kd * ldedt;
    int rpwr = kp * re + ki * reintegral + kd * rdedt;

    if (lpwr > maxSpeed) lpwr = maxSpeed;
    if (rpwr > maxSpeed) rpwr = maxSpeed;
    if (lpwr < -maxSpeed) lpwr = -maxSpeed;
    if (rpwr < -maxSpeed) rpwr = -maxSpeed;

    motors.setSpeeds(lpwr, rpwr);

    leprev = le;
    reprev = re;

    display.gotoXY(0, 0);
    display.print("L:");
    display.print(leftC);
    display.gotoXY(0, 1);
    display.print("R:");
    display.print(rightC);
  }

  motors.setSpeeds(0, 0);
}

void turnPID(int angle, bool turnLeft) {
  float kp = 0.5; 
  float ki = 0.1;
  float kd = 0.01; 
  int maxSpeed = 70;
  float iLimit = 300;

  turnSensorReset();
  float integral = 0;
  float prevError = 0;
  prevT = micros();

  while (1) {
    turnSensorUpdate();
    long currT = micros();
    float deltaT = ((float) (currT - prevT)) / 1.0e6;
    prevT = currT;

    int currentAngle = (((int32_t)turnAngle >> 16) * 360) >> 16;
    int error = angle - currentAngle;

    if (abs(error) < iLimit) {
      integral += error * deltaT;
    }

    float derivative = (error - prevError) / deltaT;
    int speed = kp * error + ki * integral + kd * derivative;

    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < -maxSpeed) speed = -maxSpeed;

    if (turnLeft) {
      motors.setSpeeds(-speed, speed);
    } else {
      motors.setSpeeds(speed, -speed);
    }

    display.gotoXY(0, 0);
    display.print("Angle:");
    display.print(currentAngle);
    display.print(F("   "));

    if (abs(error) < 5) break; 

    prevError = error;

  }

  motors.setSpeeds(0, 0);
}

void setup() {
  Serial.begin(57600);
  delay(1000);

  turnSensorSetup();

  movePID(1800);  
  turnPID(90, true);  

}

void loop() {

}
