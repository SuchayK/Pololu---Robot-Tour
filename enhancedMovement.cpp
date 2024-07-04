#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Motors motors;
Encoders encoders;
IMU imu;

const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;
const float ENCODER_CPR = 12 * 29.86F;
const float DISTANCE_PER_CLICK = WHEEL_CIRCUMFERENCE / ENCODER_CPR;
const float WHEEL_BASE = 8.5;

struct PIDController {
    float Kp, Ki, Kd;
    float integral, prevError;
    float outputMin, outputMax;
};

PIDController movePID = {0.5, 0.1, 0.2, 0, 0, -200, 200};
PIDController turnPID = {2.0, 0.1, 0.5, 0, 0, -200, 200};

float updatePID(PIDController& pid, float error, float dt) {
    pid.integral = constrain(pid.integral + error * dt, -100, 100);
    float derivative = (error - pid.prevError) / dt;
    float output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
    pid.prevError = error;
    return constrain(output, pid.outputMin, pid.outputMax);
}

void resetPID(PIDController& pid) {
    pid.integral = 0;
    pid.prevError = 0;
}

void enhancedMovePID(float targetDistance) {
    resetPID(movePID);
    int32_t targetClicks = targetDistance / DISTANCE_PER_CLICK;
    int32_t leftClicks = 0, rightClicks = 0;
    uint32_t lastTime = micros();

    while (leftClicks < targetClicks || rightClicks < targetClicks) {
        uint32_t now = micros();
        float dt = (now - lastTime) / 1e6;
        lastTime = now;

        leftClicks += encoders.getCountsAndResetLeft();
        rightClicks += encoders.getCountsAndResetRight();

        float leftError = targetClicks - leftClicks;
        float rightError = targetClicks - rightClicks;

        float leftPower = updatePID(movePID, leftError, dt);
        float rightPower = updatePID(movePID, rightError, dt);

        turnSensorUpdate();
        int32_t gyroError = (int32_t)turnAngle / turnAngle1;
        leftPower += gyroError * 0.5;
        rightPower -= gyroError * 0.5;

        motors.setSpeeds(leftPower, rightPower);

        display.clear();
        display.print(leftClicks);
        display.print(" ");
        display.print(rightClicks);

        delay(10);
    }

    motors.setSpeeds(0, 0);
}

void enhancedTurnPID(float targetAngle) {
    resetPID(turnPID);
    float initialAngle = (float)((int32_t)turnAngle / turnAngle1);
    float angleDifference = targetAngle - initialAngle;
    uint32_t lastTime = micros();

    while (abs(angleDifference) > 0.5) {
        uint32_t now = micros();
        float dt = (now - lastTime) / 1e6;
        lastTime = now;

        turnSensorUpdate();
        float currentAngle = (float)((int32_t)turnAngle / turnAngle1);
        float error = targetAngle - currentAngle;

        float turnPower = updatePID(turnPID, error, dt);

        motors.setSpeeds(-turnPower, turnPower);

        angleDifference = targetAngle - currentAngle;

        display.clear();
        display.print(currentAngle);
        display.print(" ");
        display.print(targetAngle);

        delay(10);
    }

    motors.setSpeeds(0, 0);
}

void performSmoothCurve(const Point& start, const Point& end, const Point& control1, const Point& control2) {
    const int CURVE_STEPS = 50;
    
    for (int i = 1; i <= CURVE_STEPS; i++) {
        float t = (float)i / CURVE_STEPS;
        float u = 1 - t;
        float tt = t * t;
        float uu = u * u;
        float uuu = uu * u;
        float ttt = tt * t;

        float x = uuu * start.x + 3 * uu * t * control1.x + 3 * u * tt * control2.x + ttt * end.x;
        float y = uuu * start.y + 3 * uu * t * control1.y + 3 * u * tt * control2.y + ttt * end.y;

        float dx = x - start.x;
        float dy = y - start.y;
        float distance = sqrt(dx*dx + dy*dy);
        float angle = atan2(dy, dx) * 180 / PI;

        enhancedMovePID(distance);
        enhancedTurnPID(angle);
    }
}

void updateOdometry() {
    static int16_t lastLeftCount = 0, lastRightCount = 0;
    static float x = 0, y = 0, theta = 0;

    int16_t leftCount = encoders.getCountsLeft();
    int16_t rightCount = encoders.getCountsRight();

    int16_t leftDelta = leftCount - lastLeftCount;
    int16_t rightDelta = rightCount - lastRightCount;

    float leftDistance = leftDelta * DISTANCE_PER_CLICK;
    float rightDistance = rightDelta * DISTANCE_PER_CLICK;

    float distance = (leftDistance + rightDistance) / 2;
    float rotation = (rightDistance - leftDistance) / WHEEL_BASE;

    x += distance * cos(theta + rotation / 2);
    y += distance * sin(theta + rotation / 2);
    theta += rotation;

    theta = fmod(theta + PI, 2*PI) - PI;

    lastLeftCount = leftCount;
    lastRightCount = rightCount;

    display.clear();
    display.print("X:");
    display.print(x);
    display.gotoXY(0, 1);
    display.print("Y:");
    display.print(y);
    display.gotoXY(0, 2);
    display.print("Theta:");
    display.print(theta * 180 / PI);
}