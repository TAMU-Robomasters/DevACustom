#include "subsystems/chassis.hpp"
#include "information/can_protocol.hpp"
#include "information/filters.hpp"
#include "information/pid.hpp"
#include "information/pwm_protocol.hpp"
#include "information/rc_protocol.h"
#include "init.hpp"
#include <arm_math.h>

float rX;
float rY;
float lX;
float lY;
float switch1;
float switch2;
float angle;
float angleOutput;
float magnitude;
float c1Output;
float turning;
float disp;
float motor1P;
float motor2P;
float motor3P;
float motor4P;
float currTime;
float c1SentPower;
float c1Derivative;

//INCLUDE userDebugFiles/chassis1DisplayValues.ini

namespace chassis {

chassisStates currState = notRunning;
CtrlTypes ctrlType = CURRENT;
// i don't really like this but do i care enough to change it?

filter::Kalman chassisVelFilter(0.05, 16.0, 1023.0, 0.0);

pidInstance velPidC1(pidType::velocity, 0.2, 0.001, 0.01);

chassisMotor c1Motor(userCAN::M3508_M1_ID, velPidC1, chassisVelFilter);

void task() {

    // osDelay(500);

    for (;;) {
        update();

        act();
        // set power to global variable here, message is actually sent with the others in the CAN task

        osDelay(10);
    }
}

void update() {
    if (true) {
        currState = manual;
        // will change later based on RC input and sensor based decision making
    }

    switch1 = (rcDataStruct.rc.s[0]);
		switch2 = (rcDataStruct.rc.s[1]);

    angle = atan2(getJoystick(joystickAxis::leftY), getJoystick(joystickAxis::leftX));
    angleOutput = radToDeg(angle);

    magnitude = sqrt(pow(getJoystick(joystickAxis::leftY), 2) + pow(getJoystick(joystickAxis::leftX), 2));

    currTime = HAL_GetTick();
    c1Output = c1Motor.getSpeed();

    //velPidC1.setTarget(100);

    // if button pressed on controller, change state to "followgimbal" or something
}

void act() {
    switch (currState) {
    case notRunning:
        c1Motor.setPower(0);
        break;

    case followGimbal:
        c1Motor.setPower(3);
        // obviously this will change when we have things to put here
        break;

    case manual:
        rcToPower(angle, magnitude, getJoystick(joystickAxis::rightX));
        c1Motor.setPower(velPidC1.loop(c1Motor.getSpeed()));
        c1SentPower = (c1Motor.getPower() * 163.84f);
        c1Derivative = velPidC1.getDerivative();
        // c1Motor.setPower(velPidC1.getTarget());
        // if current control, power will be set in the CAN task
        // this will change when we have things to put here
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

    disp = ((abs(magnitude) + abs(yaw)) == 0) ? 0 : abs(magnitude) / (abs(magnitude) + turnScalar*abs(yaw));
    turning = ((abs(magnitude) + abs(yaw)) == 0) ? 0 : turnScalar*abs(yaw) / (abs(magnitude) + turnScalar*abs(yaw));
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

    velPidC1.setTarget(motor1P * 200);
    // scaling max speed up to 200 rpm, can be set up to 482rpm
}

} // namespace chassis
