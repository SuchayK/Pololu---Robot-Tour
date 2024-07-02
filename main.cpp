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

void forward(int speed, int duration) {
  motors.setSpeeds(speed, speed);
  delay(duration);
  motors.setSpeeds(0, 0);
}

void turnRight(int speed, int duration) {
  motors.setSpeeds(speed, -speed);
  delay(duration);
  motors.setSpeeds(0, 0);
}

void turnLeft(int speed, int duration) {
  motors.setSpeeds(-speed, speed);
  delay(duration);
  motors.setSpeeds(0, 0);
}

void setup() {
  Serial.begin(57600);
  delay(1000);

  forward(100, 2000);   
  turnRight(100, 500);  
  forward(100, 2000);   
  turnLeft(100, 500);  
}

void loop() {
  // Empty loop
}