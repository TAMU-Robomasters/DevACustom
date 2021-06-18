#include "subsystems/chassis.hpp"
#include "information/can_protocol.hpp"
#include "information/filters.hpp"
#include "information/pid.hpp"
#include "information/pwm_protocol.hpp"
#include "information/rc_protocol.h"
#include "information/uart_protocol.hpp"
#include "init.hpp"
#include "movement/SCurveAcceleration.hpp"
#include "movement/SCurveMotionProfile.hpp"
#include <arm_math.h>

#define WHEEL_DIAM 1.62
#define WHEEL_RADIUS WHEEL_DIAM / 2

float currTime;
float angle, magnitude;
float angleOutput;
float turning, disp;
float motor1P, motor2P, motor3P, motor4P;
float c1SentPower, c1Derivative, c1Output;

double railPosition;
float lastChassisAngle;

float c1Rx;
uint8_t chassisMsg[5];

//INCLUDE userDebugFiles/chassis1DisplayValues.ini

namespace chassis {

SCurveMotionProfile::Constraints profileConstraints{1.0, 2.0, 10.0}; // m/s, m/s/s, m/s/s/s

float wheelDiameter = 1.62; // 1.62inches

chassisStates currState = notRunning;
CtrlTypes ctrlType = CURRENT;
// i don't really like this but do i care enough to change it?

filter::Kalman chassisVelFilter(0.05, 16.0, 1023.0, 0.0);

pidInstance velPidC1(pidType::velocity, 0.2, 0.000, 10.000);
pidInstance posPidChassis(pidType::position, 0.1, 0, 0);

chassisMotor c1Motor(userCAN::M3508_M1_ID, velPidC1, chassisVelFilter);

void task() {

    SCurveMotionProfile movement(profileConstraints, 1); // move 1 meter

    for (;;) {

        updateRailPosition();

        update();

        act();
        // set power to global variable here, message is actually sent with the others in the CAN task

        if (operatingType == primary) {
            osDelay(2);
        } else if (operatingType == secondary) {
            osDelay(10);
        } else
            osDelay(5);
    }
}

void update() {
    struct userUART::chassisMsgStruct* pxChassisRxedPointer;

    if (operatingType == primary) {
        currState = manual;
        // will change later based on RC input and sensor based decision making
    }

    if (operatingType == secondary) {
        currState = notRunning; // default state if not updated by primary board
        angleOutput = radToDeg(angle);

        if (userUART::chassisMsgQueue != NULL) {
            if (xQueueReceive(userUART::chassisMsgQueue, &(pxChassisRxedPointer), (TickType_t)0) == pdPASS) {
                if (pxChassisRxedPointer->prefix == userUART::d2dMsgTypes::chassis) {
                    c1Output = c1Motor.getSpeed();
                    currState = pxChassisRxedPointer->state;
                    c1Rx = pxChassisRxedPointer->m1;
                }
            }
        }
    }

    currTime = HAL_GetTick();

    //velPidC1.setTarget(100);
}

void act() {
    switch (currState) {
    case notRunning:
        c1Motor.setPower(0);
        break;

    case manual:
        if (operatingType == primary) {
            angle = atan2(getJoystick(joystickAxis::leftY), getJoystick(joystickAxis::leftX));
            magnitude = sqrt(pow(getJoystick(joystickAxis::leftY), 2) + pow(getJoystick(joystickAxis::leftX), 2));
            rcToPower(angle, magnitude, getJoystick(joystickAxis::rightX));
        }

        if (operatingType == secondary) {
            c1SentPower = (c1Motor.getPower() * 16384.0f) / 100.0f;
            c1Derivative = velPidC1.getDerivative();

            velPidC1.setTarget(c1Rx);

            // c1Motor.setPower(velPidC1.getTarget());
            c1Motor.setPower(velPidC1.loop(c1Motor.getSpeed()));
        }
        // this will change when we have things to put here
        break;

    case profiledMove:

        break;
    }
}

void rcToPower(double angle, double magnitude, double yaw) {
    // Computes the appropriate fraction of the wheel's motor power

    // Sine and cosine of math.h take angle in radians as input value
    // motor 1 back left
    // motor 2 front left
    // motor 3 front right
    // motor 4 back right
    float turnScalar = 0.6;
    int rpmScaler = 200;

    disp = ((abs(magnitude) + abs(yaw)) == 0) ? 0 : abs(magnitude) / (abs(magnitude) + turnScalar * abs(yaw));
    turning = ((abs(magnitude) + abs(yaw)) == 0) ? 0 : turnScalar * abs(yaw) / (abs(magnitude) + turnScalar * abs(yaw));
    //disp = 1 - abs(turning);
    // disp and turning represent the percentage of how much the user wants to displace or turn
    // displacement takes priority here

    float motor1Turn = yaw;
    float motor2Turn = yaw;
    float motor3Turn = yaw;
    float motor4Turn = yaw;

    float motor1Disp = (magnitude * (sin(angle) - cos(angle))) * 1;
    float motor2Disp = (magnitude * (cos(angle) + sin(angle))) * 1;
    float motor3Disp = (magnitude * (sin(angle) - cos(angle))) * -1;
    float motor4Disp = (magnitude * (cos(angle) + sin(angle))) * -1;

    motor1P = (turning * motor1Turn) + (disp * motor1Disp);
    motor2P = (turning * motor2Turn) + (disp * motor2Disp);
    motor3P = (turning * motor3Turn) + (disp * motor3Disp);
    motor4P = (turning * motor4Turn) + (disp * motor4Disp);

    //velPidC1.setTarget(motor1P * 200);
    //velPidC2.setTarget(motor2P * 200);
    //velPidC3.setTarget(motor3P * 200);
    //velPidC4.setTarget(motor4P * 200);

    // scaling max speed up to 200 rpm, can be set up to 482rpm
    motor1P *= rpmScaler;
    motor2P *= rpmScaler;
    motor3P *= rpmScaler;
    motor4P *= rpmScaler;

    sendChassisMessage(motor1P);
}

void sendChassisMessage(float m1) {
    int16_t m1S = static_cast<int16_t>(m1 * 50);
    chassisMsg[0] = 'c';
    chassisMsg[1] = static_cast<uint8_t>(currState);
    chassisMsg[2] = (m1S + 32768) >> 8;
    chassisMsg[3] = (m1S + 32768);
    chassisMsg[4] = 'e';
    HAL_UART_Transmit(&huart8, (uint8_t*)chassisMsg, sizeof(chassisMsg), 1);
}

void updateRailPosition() {
    //delta is in Radians
    float deltaChas = gimbal::calculateAngleError(c1Motor.getAngle(), lastChassisAngle) / 19.0f * 22 / 9 * (WHEEL_RADIUS);
    lastChassisAngle = c1Motor.getAngle();
    railPosition += deltaChas;
}

} // namespace chassis
