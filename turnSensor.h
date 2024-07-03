#include <Wire.h>

const int32_t turnAngle45 = 0x20000000;

const int32_t turnAngle90 = turnAngle45 * 2;

const int32_t turnAngle1 = (turnAngle45 + 22) / 45;

uint32_t turnAngle = 0;

int16_t turnRate;

int16_t gyroOffset;

uint16_t gyroLastUpdate = 0;


void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}


void turnSensorUpdate()
{
  
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;
  
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  int32_t d = (int32_t)turnRate * dt;

  turnAngle += (int64_t)d * 14680064 / 17578125;
}

void turnSensorSetup()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  display.clear();
  display.print(F("Gyro cal"));

  ledYellow(1);

  delay(500);

  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) 
  {

    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  display.clear();
  turnSensorReset();

  turnSensorUpdate();
  display.gotoXY(0, 0);
  display.print((((int32_t)turnAngle >> 16) * 360) >> 16);
  display.print(F("   "));

  display.clear();
}