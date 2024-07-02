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
const float WHEEL_CIRCUMFERENCE = 10.05;

volatile int leftC = 0;
volatile int rightC = 0;

void movePID(int target) {
  float kp = 0.1; 
  int maxSpeed = 100;

  leftC = 0;
  rightC = 0;
  while (leftC < target && rightC < target) {
    leftC += encoders.getCountsAndResetLeft();
    rightC += encoders.getCountsAndResetRight();

    int le = target - leftC;
    int re = target - rightC;

    int lpwr = kp * le;
    int rpwr = kp * re;

    if (lpwr > maxSpeed) lpwr = maxSpeed;
    if (rpwr > maxSpeed) rpwr = maxSpeed;

    motors.setSpeeds(lpwr, rpwr);

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
  int maxSpeed = 50;

  turnSensorReset();

  while (1) {
    turnSensorUpdate();
    int error = angle - (((int32_t)turnAngle >> 16) * 360) >> 16;

    int speed = kp * error;
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < -maxSpeed) speed = -maxSpeed;

    if (turnLeft) {
      motors.setSpeeds(-speed, speed);
    } else {
      motors.setSpeeds(speed, -speed);
    }

    display.gotoXY(0, 0);
    display.print("Angle:");
    display.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    display.print(F("   "));

    if (abs(error) < 5) break;
  }

  motors.setSpeeds(0, 0);
}

void setup() {
  Serial.begin(7600);
  delay(1000);

  turnSensorSetup();

  movePID(1800);   
  turnPID(90, true);  

}

void loop() {
  // Empty loop
}