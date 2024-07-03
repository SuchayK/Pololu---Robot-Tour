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
  float Kc = 0.35;
  int maxSpeed = 100;
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

    if (leftC < rightC) {
      lpwr += Kc * (rightC - leftC);
    } else if (rightC < leftC) {
      rpwr += Kc * (leftC - rightC);
    }

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


void rightPID() {

  float lu = 1;
  float ru = 1;

  while (lu != 0 || ru != 0) {
    leftC += encoders.getCountsAndResetLeft();
    rightC += encoders.getCountsAndResetRight();

    int ltarget = 235;
    int rtarget = -235;

    int diff = leftC - rightC;

    float kp = 0.07;
    float ki = 0;
    float kd = 0.05;

    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    int pos1 = leftC;
    int pos2 = rightC;

    int le = ltarget - pos1;
    int re = rtarget - pos2;

    float ldedt = (le-leprev)/(deltaT);
    float rdedt = (re-reprev)/(deltaT);

    leintegral = leintegral + le*deltaT;
    reintegral = reintegral + re*deltaT;

    lu = kp*le + kd*ldedt + ki*leintegral;
    ru = kp*re + kd*rdedt + ki*reintegral;

    float lpwr = lu;

    if( lpwr > 55 ){
      lpwr = 55;
    }

    if ( lpwr < 40 && lpwr > 0) {
      lpwr = 40;
    }

    if ( lpwr < 0 ){
      lpwr = -25;
    }

    float rpwr = ru;

    if( rpwr < -55){
      rpwr = -55;
    }

    if ( rpwr > -40 && rpwr < 0) {
      rpwr = -40;
    }

    if ( rpwr > 0 ){
      rpwr = 25;
    }

    motors.setSpeeds(lpwr, rpwr);

    leprev = le;
    reprev = re;

    turnSensorUpdate();

    // print left pwr value
    display.gotoXY(0, 0);
    display.print(lpwr);
    display.print(F("   "));

    // print right pwr value
    display.gotoXY(0, 1);
    display.print(rpwr);
    display.print(F("   "));

    // print diff in encoders
    // display.gotoXY(0, 0);
    // display.print(diff);
    // display.print(F("   "));

  }

  motors.setSpeeds(0, 0);

  leftC = 0;
  rightC = 0;
  prevT = 0;
  leprev = 0;
  leintegral = 0;
  reprev = 0;
  reintegral = 0;

  delay(800);

}

void leftPID() {

  float lu = -1;
  float ru = 1;

  while (lu < 0 || ru > 0) {
    leftC += encoders.getCountsAndResetLeft();
    rightC += encoders.getCountsAndResetRight();

    int ltarget = -237;
    int rtarget = 237;

    int diff = leftC + rightC;

    float kp = 0.07;
    float ki = 0.02;
    float kd = 0.001;
    float liLimit = -50;
    float riLimit = 50;

    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    int pos1 = leftC;
    int pos2 = rightC;

    int le = ltarget - pos1;
    int re = rtarget - pos2;

    float ldedt = (le-leprev)/(deltaT);
    float rdedt = (re-reprev)/(deltaT);

    if (le > liLimit){
      leintegral = leintegral + le*deltaT;
    }

    if (re < riLimit){
      reintegral = reintegral + re*deltaT;
    }

    lu = kp*le + kd*ldedt + ki*leintegral;
    ru = kp*re + kd*rdedt + ki*reintegral;

    float lpwr = lu;

    if( lpwr < -55 ){
      lpwr = -55;
    }

    if ( lpwr > -32 && lpwr < 0) {
      lpwr = -32;
    }

    float rpwr = ru;

    if( rpwr > 55){
      rpwr = 55;
    }

    if ( rpwr < 32 && rpwr > 0) {
      rpwr = 32;
    }

    motors.setSpeeds(lpwr, rpwr);

    leprev = le;
    reprev = re;

    turnSensorUpdate();
    
    // print left pwr value
    display.gotoXY(0, 0);
    display.print(lpwr);
    display.print(F("   "));

    // print right pwr value
    display.gotoXY(0, 1);
    display.print(rpwr);
    display.print(F("   "));

    // print diff in encoders
    // display.gotoXY(0, 0);
    // display.print(diff);
    // display.print(F("   "));
  }

  motors.setSpeeds(0, 0);

  leftC = 0;
  rightC = 0;
  prevT = 0;
  leprev = 0;
  leintegral = 0;
  reprev = 0;
  reintegral = 0;

  display.clear();

  delay(500);

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
