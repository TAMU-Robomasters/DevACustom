#include "subsystems/chassis.hpp"
#include "imu/imu_protocol.hpp"
#include "information/can_protocol.hpp"
#include "information/filters.hpp"
#include "information/pid.hpp"
#include "information/pwm_protocol.hpp"
#include "information/rc_protocol.h"
#include "information/uart_protocol.hpp"
#include "init.hpp"
#include <arm_math.h>

float currTime;
float angle, magnitude;
float angleOutput;
float turning, disp;
float motor1P, motor2P, motor3P, motor4P = 0;
float c1SentPower, c1Derivative, c1Output;
float chasMaxRPM = 200;

float c1Rx, c2Rx, c3Rx, c4Rx;
uint8_t chassisMsg[7];
uint8_t chassisPowerMsg[3];

int btnCToggleState = 0;

uint8_t chasStateShow;

//INCLUDE userDebugFiles/chassis1DisplayValues.ini

namespace chassis {

chassisStates currState = notRunning;
CtrlTypes ctrlType = CURRENT;
// i don't really like this but do i care enough to change it?

filter::Kalman chassisVelFilter(0.05, 16.0, 1023.0, 0.0);

// pidInstance chassisTurnPID(pidType::position, 0.2, 0.000, 0.000);

pidInstance velPidC1(pidType::velocity, 0.2, 0.000, 2.000);
pidInstance velPidC2(pidType::velocity, 0.2, 0.000, 2.000);
pidInstance velPidC3(pidType::velocity, 0.2, 0.000, 2.000);
pidInstance velPidC4(pidType::velocity, 0.2, 0.000, 2.000);

chassisMotor c1Motor(userCAN::M3508_M1_ID, velPidC1, chassisVelFilter);
chassisMotor c2Motor(userCAN::M3508_M2_ID, velPidC2, chassisVelFilter);
chassisMotor c3Motor(userCAN::M3508_M3_ID, velPidC3, chassisVelFilter);
chassisMotor c4Motor(userCAN::M3508_M4_ID, velPidC4, chassisVelFilter);

void task() {

    for (;;) {
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
        /*if (getSwitch(switchType::right) == switchPosition::up) {
            currState = spinToWin;
        } else */
        if (fabs(getJoystick(joystickAxis::leftX)) > 0 || fabs(getJoystick(joystickAxis::leftY)) > 0 || fabs(getJoystick(joystickAxis::rightX)) > 0) {
            currState = manual;
        }
				// if (getBtn(btnW) || getBtn(btnA) || getBtn(btnS) || getBtn(btnD)){
				// 		currState = manual;
				// }
				else {
						currState = notRunning;
				}

        if (btnIsRising(btnC)) {
            chassisPowerUpdate();
        }
    }

    if (operatingType == secondary) {
        c1Rx = c2Rx = c3Rx = c4Rx = 0;
        // currState = notRunning; // default state if not updated by primary board
        angleOutput = radToDeg(angle);

        if (userUART::chassisMsgQueue != NULL) {
            if (xQueueReceive(userUART::chassisMsgQueue, &(pxChassisRxedPointer), (TickType_t)0) == pdPASS) {
                if (pxChassisRxedPointer->prefix == userUART::d2dMsgTypes::chassis) {
                    c1Output = c1Motor.getSpeed();
                    currState = pxChassisRxedPointer->state;
                    c1Rx = pxChassisRxedPointer->m1;
                    c2Rx = pxChassisRxedPointer->m2;
                    c3Rx = pxChassisRxedPointer->m3;
                    c4Rx = pxChassisRxedPointer->m4;
                }
            }
        }
    }

    chasStateShow = currState;

    //currTime = HAL_GetTick();

    //velPidC1.setTarget(100);

    // if button pressed on controller, change state to "followgimbal" or something
}

void act() {
    switch (currState) {
    case notRunning:
        c1Motor.setPower(0);
        c2Motor.setPower(0);
        c3Motor.setPower(0);
        c4Motor.setPower(0);
        if (operatingType == primary) {
            sendChassisMessage(0, 0, 0, 0);
        }
        break;

    case followGimbal:
        c1Motor.setPower(0);
        c2Motor.setPower(0);
        c3Motor.setPower(0);
        c4Motor.setPower(0);
        if (operatingType == primary) {
            sendChassisMessage(0, 0, 0, 0);
        }
        if (operatingType == secondary) {
        }
        // this will change when we have things to put here
        break;

    case manual:
        if (operatingType == primary) {
            angle = atan2(getJoystick(joystickAxis::leftY), getJoystick(joystickAxis::leftX));
            magnitude = sqrt(pow(getJoystick(joystickAxis::leftY), 2) + pow(getJoystick(joystickAxis::leftX), 2));
            rcToPower(getJoystick(joystickAxis::rightX), getJoystick(joystickAxis::leftY), getJoystick(joystickAxis::leftX));
            // rcToPower(getMouse(x)/100, getBtn(btnW) - getBtn(btnS), getBtn(btnA) - getBtn(btnD));
        }

        if (operatingType == secondary) {
            c1SentPower = (c1Motor.getPower() * 16.384f);
            c1Derivative = velPidC1.getDerivative();

            velPidC1.setTarget(c1Rx);
            velPidC2.setTarget(c2Rx);
            velPidC3.setTarget(c3Rx);
            velPidC4.setTarget(c4Rx);

            // c1Motor.setPower(velPidC1.getTarget());
            c1Motor.setPower(velPidC1.loop(c1Motor.getSpeed()));
            c2Motor.setPower(velPidC2.loop(c2Motor.getSpeed()));
            c3Motor.setPower(velPidC3.loop(c3Motor.getSpeed()));
            c4Motor.setPower(velPidC4.loop(c4Motor.getSpeed()));
        }
        // this will change when we have things to put here
        break;

    case spinToWin:
        if (operatingType == primary) {
            rcToPower(0.625, 0, 0);
        }

        if (operatingType == secondary) {
            c1SentPower = (chassisPowerLimit / 24) * (M3508_MAX_CURRENT / M3508_MAX_AMPS) * c1Motor.getPower() / 100;
            c1Derivative = velPidC1.getDerivative();

            velPidC1.setTarget(c1Rx);
            velPidC2.setTarget(c2Rx);
            velPidC3.setTarget(c3Rx);
            velPidC4.setTarget(c4Rx);

            // c1Motor.setPower(velPidC1.getTarget());
            c1Motor.setPower(velPidC1.loop(c1Motor.getSpeed()));
            c2Motor.setPower(velPidC2.loop(c2Motor.getSpeed()));
            c3Motor.setPower(velPidC3.loop(c3Motor.getSpeed()));
            c4Motor.setPower(velPidC4.loop(c4Motor.getSpeed()));
        }
    }
}

void rcToPower(double rotate, double forward, double strafe) {
    // Computes the appropriate fraction of the wheel's motor power

    // Sine and cosine of math.h take angle in radians as input value
    // motor 1 back left
    // motor 2 front left
    // motor 3 front right
    // motor 4 back right
    float turnBias = 0.7;

    rotate *= turnBias;

    motor1P = forward + rotate - strafe;
    motor2P = forward + rotate + strafe;
    motor3P = -(forward - rotate - strafe);
    motor4P = -(forward - rotate + strafe);

    float max = fabs(motor1P);
    max = fabs(motor2P) > max ? fabs(motor2P) : max;
    max = fabs(motor3P) > max ? fabs(motor3P) : max;
    max = fabs(motor4P) > max ? fabs(motor4P) : max;

    if (max > 1) {
        motor1P /= max;
        motor2P /= max;
        motor3P /= max;
        motor4P /= max;
    }

    // scaling max speed up to 200 rpm, can be set up to 482rpm
    motor1P *= chasMaxRPM;
    motor2P *= chasMaxRPM;
    motor3P *= chasMaxRPM;
    motor4P *= chasMaxRPM;

    sendChassisMessage(motor1P, motor2P, motor3P, motor4P);
}

void sendChassisMessage(float m1, float m2, float m3, float m4) {
    float valScaler = chasMaxRPM / int8_MAX;
    chassisMsg[0] = 'c';
    chassisMsg[1] = static_cast<uint8_t>(currState);
    chassisMsg[2] = static_cast<uint8_t>((m1 / valScaler) + int8_MAX);
    chassisMsg[3] = static_cast<uint8_t>((m2 / valScaler) + int8_MAX);
    chassisMsg[4] = static_cast<uint8_t>((m3 / valScaler) + int8_MAX);
    chassisMsg[5] = static_cast<uint8_t>((m4 / valScaler) + int8_MAX);
    chassisMsg[6] = 'e';
    HAL_UART_Transmit(&huart8, (uint8_t*)chassisMsg, sizeof(chassisMsg), 1);
}

void sendChassisPowerMessage(float power) {
    chassisPowerMsg[0] = 'x';
    chassisPowerMsg[1] = static_cast<uint8_t>(power);
    chassisPowerMsg[2] = 'e';
    HAL_UART_Transmit(&huart8, (uint8_t*)chassisPowerMsg, sizeof(chassisPowerMsg), 1);
}

void chassisPowerUpdate() {
    btnCToggleState = btnCToggleState == 3 ? btnCToggleState = 0 : btnCToggleState++;
    switch (btnCToggleState) {
    case 0:
        sendChassisPowerMessage(40);
        break;

    case 1:
        sendChassisPowerMessage(45);
        break;

    case 2:
        sendChassisPowerMessage(50);
        break;

    case 3:
        sendChassisPowerMessage(55);
        break;
    }
}

} // namespace chassis
