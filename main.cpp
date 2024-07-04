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
long prevT = 0;
float leprev = 0;
float leintegral = 0;
float reprev = 0;
float reintegral = 0;

const int32_t turnAngle45 = 0x20000000;
const int32_t turnAngle90 = turnAngle45 * 2;
const int32_t turnAngle1 = (turnAngle45 + 22) / 45;
uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;

float x = 0, y = 0, theta = 0;
float leftDistance = 0, rightDistance = 0;
const float WHEEL_BASE = 8.5;

void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

void turnSensorUpdate() {
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;
  int32_t d = (int32_t)turnRate * dt;
  turnAngle += (int64_t)d * 14680064 / 17578125;
}

void turnSensorSetup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();
  display.clear();
  display.print(F("Gyro cal"));
  ledYellow(1);
  delay(500);
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    while(!imu.gyroDataReady()) {}
    imu.readGyro();
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;
  display.clear();
  turnSensorReset();
  turnSensorUpdate();
  display.clear();
}

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

    updateOdometry();

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
    display.gotoXY(0, 0);
    display.print(diff);
    display.print(F("   "));

    updateOdometry();

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
    display.gotoXY(0, 0);
    display.print(diff);
    display.print(F("   "));

    updateOdometry();

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

void updateOdometry() {

  int16_t leftCounts = encoders.getCountsAndResetLeft();
  int16_t rightCounts = encoders.getCountsAndResetRight();
  
  float leftDist = (leftCounts * WHEEL_CIRCUMFERENCE) / (CLICKS_PER_ROTATION * GEAR_RATIO);
  float rightDist = (rightCounts * WHEEL_CIRCUMFERENCE) / (CLICKS_PER_ROTATION * GEAR_RATIO);
  
  float distanceTraveled = (leftDist + rightDist) / 2.0;
  float deltaTheta = (rightDist - leftDist) / WHEEL_BASE;
  
  x += distanceTraveled * cos(theta + deltaTheta / 2.0);
  y += distanceTraveled * sin(theta + deltaTheta / 2.0);
  theta += deltaTheta;
  
  while (theta > 2 * PI) theta -= 2 * PI;
  while (theta < -2 * PI) theta += 2 * PI;
  
  leftDistance += leftDist;
  rightDistance += rightDist;

}

void displayOdometry() {

  display.clear();
  display.gotoXY(0, 0);
  display.print("X:");
  display.print(x);
  display.gotoXY(0, 1);
  display.print("Y:");
  display.print(y);
  display.gotoXY(0, 2);
  display.print("Theta:");
  display.print(theta * 180 / PI);

}

void performSmoothTurn(Point start, Point end, Point control1, Point control2) {

    std::vector<Point> controlPoints = {start, control1, control2, end};
    BezierCurve curve(controlPoints);
    std::vector<Point> pathPoints = curve.generateCurvePoints(100);

    for (size_t i = 1; i < pathPoints.size(); ++i) {
        Point current = pathPoints[i];
        Point previous = pathPoints[i - 1];
        float dx = current.x - previous.x;
        float dy = current.y - previous.y;
        float angle = atan2(dy, dx);
        float distance = sqrt(dx*dx + dy*dy);
        
 
        int targetCounts = (distance * CLICKS_PER_ROTATION * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;
        
        movePID(targetCounts);
        
        turnPID(angle * 180 / PI, true);

    }

}

void autonomousRoutine2() {

  display.clear();
  display.print("Auto 2 Start");
  delay(1000);

  Point currentPos = {0, 0};
  float currentAngle = 0; 

  for (int i = 1; i <= 4; i++) {

    movePID(500 * i);
    currentPos.x += 50 * i;
    turnPID(90, true);
    currentAngle -= 90;
    movePID(500 * i);
    currentPos.y += 50 * i;
    turnPID(90, true);
    currentAngle -= 90;

  }

  for (int i = 0; i < 3; i++) {

    movePID(1500);
    currentPos.x -= 150;
    turnPID(45, false);
    currentAngle += 45;
    movePID(707); 
    currentPos.x -= 50;
    currentPos.y += 50;
    turnPID(90, true);
    currentAngle -= 90;
    movePID(707);
    currentPos.x -= 50;
    currentPos.y -= 50;
    turnPID(45, false);
    currentAngle += 45;

  }

  Point center1 = {currentPos.x - 75, currentPos.y};
  Point center2 = {currentPos.x + 75, currentPos.y};

  for (int i = 0; i < 2; i++) {

    Point end1 = {center1.x, center1.y - 75};
    Point control1 = {center1.x - 75, center1.y};
    Point control2 = {center1.x - 75, center1.y - 75};
    performSmoothTurn(currentPos, end1, control1, control2);
    currentPos = end1;

    Point end2 = {center1.x, center1.y + 75};
    Point control3 = {center1.x + 75, center1.y - 75};
    Point control4 = {center1.x + 75, center1.y};
    performSmoothTurn(currentPos, end2, control3, control4);
    currentPos = end2;

    Point end3 = {center2.x, center2.y + 75};
    Point control5 = {center2.x - 75, center2.y};
    Point control6 = {center2.x - 75, center2.y + 75};
    performSmoothTurn(currentPos, end3, control5, control6);
    currentPos = end3;

    Point end4 = {center2.x, center2.y - 75};
    Point control7 = {center2.x + 75, center2.y + 75};
    Point control8 = {center2.x + 75, center2.y};
    performSmoothTurn(currentPos, end4, control7, control8);
    currentPos = end4;

  }

  turnPID(calculateAngle(currentPos, {0, 0}), true);
  currentAngle = calculateAngle(currentPos, {0, 0});
  movePID(calculateDistance(currentPos, {0, 0}));
  currentPos = {0, 0};

  turnPID(-currentAngle, true);
  currentAngle = 0;

  display.clear();
  display.print("Auto 2 End");
  display.gotoXY(0, 1);
  display.print("X:");
  display.print(currentPos.x);
  display.print(" Y:");
  display.print(currentPos.y);
  delay(5000);

}

float calculateAngle(Point start, Point end) {

    return atan2(end.y - start.y, end.x - start.x) * 180 / PI;

}

int calculateDistance(Point start, Point end) {

    float dx = end.x - start.x;
    float dy = end.y - start.y;
    return sqrt(dx*dx + dy*dy) * (CLICKS_PER_ROTATION * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;

}

void autonomousRoutine3() {

  display.clear();
  display.print("Auto 3 Start");
  delay(1000);

  Point currentPos = {0, 0};
  float currentAngle = 0;  

  for (int i = 0; i < 4; i++) {

    movePID(1414); 
    currentPos.x += 100 * cos(currentAngle * PI / 180);
    currentPos.y += 100 * sin(currentAngle * PI / 180);
    turnPID(90, true);
    currentAngle -= 90;

  }

  for (int i = 0; i < 4; i++) {

    Point start = currentPos;
    Point end = {

        currentPos.x + 150 * cos((currentAngle + 90) * PI / 180),
        currentPos.y + 150 * sin((currentAngle + 90) * PI / 180)

    };
    Point control1 = {

        currentPos.x + 75 * cos(currentAngle * PI / 180),
        currentPos.y + 75 * sin(currentAngle * PI / 180)
        
    };
    Point control2 = {

        end.x - 75 * cos(currentAngle * PI / 180),
        end.y - 75 * sin(currentAngle * PI / 180)

    };

    performSmoothTurn(start, end, control1, control2);
    currentPos = end;
    turnPID(180, false);
    currentAngle += 180;
    
  }

  int sideLength = 500;

  for (int i = 0; i < 4; i++) {

    for (int j = 0; j < 4; j++) {

      movePID(sideLength);
      currentPos.x += sideLength * cos(currentAngle * PI / 180) / 10;
      currentPos.y += sideLength * sin(currentAngle * PI / 180) / 10;
      turnPID(90, false);
      currentAngle += 90;

    }

      sideLength -= 100;

  }

  for (int i = 0; i < 4; i++) {

    for (int j = 0; j < 20; j++) {

      Point start = currentPos;
      
      Point end = {

          currentPos.x + 50,
          currentPos.y + 50 * sin(j * PI / 10)

      };

      Point control1 = {start.x + 25, start.y};
      Point control2 = {end.x - 25, end.y};
      performSmoothTurn(start, end, control1, control2);
      currentPos = end;

    }

    turnPID(180, true);
    currentAngle -= 180;
    
  }

  while (calculateDistance(currentPos, {0, 0}) > 100) {

    Point start = currentPos;
    Point end = {

      (currentPos.x * 0.9),
      (currentPos.y * 0.9)

    }

    Point control1 = {

      start.x - (start.y - end.y) * 0.5,
      start.y + (start.x - end.x) * 0.5

    }

    Point control2 = {

      end.x - (start.y - end.y) * 0.5,
      end.y + (start.x - end.x) * 0.5

    }

    performSmoothTurn(start, end, control1, control2);
    currentPos = end;

  }

  turnPID(calculateAngle(currentPos, {0, 0}), true);
  currentAngle = calculateAngle(currentPos, {0, 0});
  movePID(calculateDistance(currentPos, {0, 0}));
  currentPos = {0, 0};

  turnPID(-currentAngle, true);
  currentAngle = 0;

  display.clear();
  display.print("Auto 3 End");
  display.gotoXY(0, 1);
  display.print("X:");
  display.print(currentPos.x);
  display.print(" Y:");
  display.print(currentPos.y);
  delay(5000);

}

void setup() {

  Serial.begin(57600);
  delay(1000);

  turnSensorSetup();
  encoders.init();

  autonomousRoutine2();
  autonomousRoutine3();

  // Point currentPos = {0, 0};
  // float currentAngle = 0; 

  // movePID(1800);
  // currentPos.x += 180;

  // Point end1 = {280, 100};
  // Point control1 = {230, 0};
  // Point control2 = {280, 50};
  // performSmoothTurn(currentPos, end1, control1, control2);
  // currentPos = end1;

  // movePID(1000);
  // currentPos.y += 100;

  // turnPID(90, false);
  // currentAngle += 90;

  // movePID(1500);
  // currentPos.x -= 150;

  // Point end2 = {80, 300};
  // Point control3 = {80, 250};
  // Point control4 = {130, 300};
  // performSmoothTurn(currentPos, end2, control3, control4);
  // currentPos = end2;

  // movePID(1200);
  // currentPos.y += 120;

  // turnPID(90, true);
  // currentAngle -= 90;

  // movePID(2000);
  // currentPos.x += 200;

  // Point end3 = {330, 500};
  // Point control5 = {330, 420};
  // Point control6 = {280, 500};
  // performSmoothTurn(currentPos, end3, control5, control6);
  // currentPos = end3;

  // movePID(1000);
  // currentPos.y += 100;

  // turnPID(90, true);
  // currentAngle -= 90;

  // movePID(1500);
  // currentPos.x += 150;

  // Point end4 = {530, 550};
  // Point control7 = {480, 600};
  // Point control8 = {530, 600};
  // performSmoothTurn(currentPos, end4, control7, control8);
  // currentPos = end4;

  // movePID(1000);
  // currentPos.y -= 100;

  // turnPID(90, false);
  // currentAngle += 90;

  // movePID(2000);
  // currentPos.x += 200;

  // Point end5 = {730, 450};
  // Point control9 = {730, 500};
  // Point control10 = {680, 450};
  // performSmoothTurn(currentPos, end5, control9, control10);
  // currentPos = end5;

  // movePID(1000);
  // currentPos.y -= 100;

  // display.clear();
  // display.print("Final X: ");
  // display.print(currentPos.x);
  // display.gotoXY(0, 1);
  // display.print("Final Y: ");
  // display.print(currentPos.y);

  // delay(5000);

}


void loop() {

  updateOdometry();
  displayOdometry();
  delay(100);

}
