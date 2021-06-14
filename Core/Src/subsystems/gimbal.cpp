#include "subsystems/gimbal.hpp"
#include "imu/imu_protocol.hpp"
#include "information/rc_protocol.h"
#include "information/uart_protocol.hpp"
#include "init.hpp"

float32_t bung = 0;
float yawAngleShow;
double yawTarget;
float yawTargetShow;
float yawPidShow;
float yawDerivShow;
float pitchAngleShow;
double pitchTarget;
float pitchTargetShow;
float pitchPidShow;
float currGimbTime, lastGimbTime, lastGimbLoopTime;
float pitchPowerShow;
float dispYaw;
float dispPitch;
float roll, pitch, yaw;

int stateShow;

float rcRightX;
float rcRightY;

float yawSave = degToRad(94.0);
float pitchSave = degToRad(355.0);
float imuGimbYawSave = degToRad(180.0);
float imuGimbPitchSave = degToRad(0.0);

float yawRx = 0;
float gimbYawIMU = 0;
uint8_t gimbMsg[7];

int16_t test;
uint8_t test1, test2;

namespace gimbal {

gimbalStates currState = notRunning;
CtrlTypes ctrlType = VOLTAGE;

filter::Kalman gimbalVelFilter(0.05, 16.0, 1023.0, 0.0);

// pidInstance yawPosPid(pidType::position, 95.0, 0.00, 2100.00);
// pidInstance pitchPosPid(pidType::position, 350.0, 0.00, 6000.0); // not using direct motor speeds
pidInstance yawPosPid(pidType::position, 150.0, 0.0, 0.6);
pidInstance pitchPosPid(pidType::position, 350.0, 0.00, 1.0);
float kF = 0;

gimbalMotor yawMotor(userCAN::GM6020_YAW_ID, yawPosPid, gimbalVelFilter);
gimbalMotor pitchMotor(userCAN::GM6020_PIT_ID, pitchPosPid, gimbalVelFilter);

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
    //currState = notRunning;
    struct userUART::aimMsgStruct* pxAimRxedPointer;
    struct userUART::gimbMsgStruct* pxGimbRxedPointer;

    userIMU::imuUpdate();

    roll = userIMU::imuRoll();
    pitch = userIMU::imuPitch();
    yaw = userIMU::imuYaw();

    if (operatingType == primary) {
        currState = notRunning; // default state

        pitchAngleShow = radToDeg(pitchMotor.getAngle());
        pitchTargetShow = radToDeg(pitchPosPid.getTarget());
        pitchPidShow = pitchPosPid.getOutput();
        rcRightX = getJoystick(joystickAxis::rightX);
        rcRightY = getJoystick(joystickAxis::rightY);

        if (fabs(getJoystick(joystickAxis::rightX)) + fabs(getJoystick(joystickAxis::rightY)) >= 0.05f) {
            currState = rc;
        }

        if (userUART::aimMsgQueue != NULL) {
            if (xQueueReceive(userUART::aimMsgQueue, &(pxAimRxedPointer), (TickType_t)0) == pdPASS) {
                if (pxAimRxedPointer->prefix == userUART::jetsonMsgTypes::aimAt) {
                    dispYaw = pxAimRxedPointer->disp[0];
                    dispPitch = pxAimRxedPointer->disp[1];
                    //currState = aimFromCV;
                }
            }
        }
    }

    if (operatingType == secondary) {
        currState = notRunning;

        yawAngleShow = radToDeg(yawMotor.getAngle());
        yawPidShow = yawPosPid.getOutput();

        if (userUART::gimbMsgQueue != NULL) {
            if (xQueueReceive(userUART::gimbMsgQueue, &(pxGimbRxedPointer), (TickType_t)0) == pdPASS) {
                if (pxGimbRxedPointer->prefix == userUART::d2dMsgTypes::gimbal) {
                    currState = pxGimbRxedPointer->state;
                    yawRx = pxGimbRxedPointer->rx;
                    gimbYawIMU = pxGimbRxedPointer->imuY;
                }
            }
        }
    }

    stateShow = currState;

    //if (yawMotor.getAngle() + dispYaw > degToRad(180.0) || yawMotor.getAngle() + dispYaw < degToRad(60.0)) {
    //    currState = idle;
    //}
    //if (pitchMotor.getAngle() + dispPitch > degToRad(180.0) || pitchMotor.getAngle() + dispPitch < degToRad(90.0)) {
    //    currState = idle;
    //}
}

void act() {
    switch (currState) {
    case notRunning:
        pitchMotor.setPower(0);
        yawMotor.setPower(0);
        if (operatingType == primary) {
            imuGimbPitchSave = userIMU::imuPitch();
            sendGimbMessage(0);
        }
        break;

    case aimFromCV:
        // gimbal motors controlled through voltage, sent messages over CAN
        // yawTarget = -calculateAngleError(yawMotor.getAngle(), yawMotor.getAngle() - dispYaw);
        // yawTarget = -calculateAngleError(yawMotor.getAngle(), degToRad(274.0));
        // pitchTarget = -calculateAngleError(pitchMotor.getAngle(), degToRad(21.5));
        // yawMotor.setPower(yawPosPid.loop(yawTarget));
        // pitchMotor.setPower(-kF * cos(normalizePitchAngle()));
        if (operatingType == primary) {
            pitchSave = pitchMotor.getAngle();
            imuGimbPitchSave = userIMU::imuPitch();

            pitchPosPid.setTarget(0.0);
            pitchTarget = -calculateAngleError(pitchMotor.getAngle(), pitchMotor.getAngle() - dispPitch);
            pitchMotor.setPower(pitchPosPid.loop(pitchTarget, pitchMotor.getSpeed()) + (-kF * cos(normalizePitchAngle())));
            sendGimbMessage(dispYaw);
        }
        if (operatingType == secondary) {
            yawSave = yawMotor.getAngle();
            imuGimbYawSave = gimbYawIMU;

            yawPosPid.setTarget(0.0);
            yawTarget = -calculateAngleError(yawMotor.getAngle(), yawMotor.getAngle() - yawRx);
            yawMotor.setPower(yawPosPid.loop(yawTarget, yawMotor.getSpeed()));
        }
        break;

    case idle:
        // yawTarget = -calculateAngleError(yawMotor.getAngle(), yawSave);
        // yawTarget = -calculateAngleError(yawMotor.getAngle(), degToRad(274.0));
        // pitchTarget = -calculateAngleError(pitchMotor.getAngle(), degToRad(21.5));
        // pitchMotor.setPower(-kF * cos(normalizePitchAngle()));
        if (operatingType == primary) {
            imuGimbPitchSave = userIMU::imuPitch();

            pitchPosPid.setTarget(0.0);
            pitchTarget = -calculateAngleError(pitchMotor.getAngle(), pitchSave);
            pitchMotor.setPower(pitchPosPid.loop(pitchTarget, pitchMotor.getSpeed()) + (-kF * cos(normalizePitchAngle())));
            sendGimbMessage(0);
        }
        if (operatingType == secondary) {
            imuGimbYawSave = gimbYawIMU;

            yawPosPid.setTarget(0.0);
            yawDerivShow = yawMotor.getSpeed() * yawPosPid.getkD();
            yawTarget = -calculateAngleError(yawMotor.getAngle(), yawSave);
            yawMotor.setPower(yawPosPid.loop(yawTarget, yawMotor.getSpeed()));
        }
        break;

    case imuIdle:
        if (operatingType == primary) {
            pitchPosPid.setTarget(0.0);
            pitchTarget = calculateAngleError(userIMU::imuPitch(), imuGimbPitchSave);
            pitchMotor.setPower(pitchPosPid.loop(pitchTarget, pitchMotor.getSpeed()) + (-kF * cos(normalizePitchAngle())));
            sendGimbMessage(0);
        }
        if (operatingType == secondary) {
            yawPosPid.setTarget(0.0);
            yawDerivShow = yawMotor.getSpeed() * yawPosPid.getkD();
            yawTarget = calculateAngleError(gimbYawIMU, imuGimbYawSave);
            yawMotor.setPower(yawPosPid.loop(yawTarget, yawMotor.getSpeed()));
        }
        break;

    case rc:
        if (operatingType == primary) {
            pitchSave = pitchMotor.getAngle();
            pitchPosPid.setTarget(0.0);
            dispPitch = getJoystick(joystickAxis::rightY) / 10.0f;

            pitchTarget = -calculateAngleError(pitchMotor.getAngle(), pitchMotor.getAngle() - dispPitch);
            pitchMotor.setPower(pitchPosPid.loop(pitchTarget, pitchMotor.getSpeed()) + (-kF * cos(normalizePitchAngle())));

            sendGimbMessage(0);
            // if (getJoystick(joystickAxis::rightX) != 0){
            // dispYaw = -getJoystick(joystickAxis::rightX) / 6.0f;
            // sendGimbMessage(dispYaw);
            // }
        }
        if (operatingType == secondary) {
            imuGimbYawSave = gimbYawIMU;

            yawPosPid.setTarget(0.0);
            yawDerivShow = yawMotor.getSpeed() * yawPosPid.getkD();
            yawTarget = -calculateAngleError(yawMotor.getAngle(), yawSave);
            yawMotor.setPower(yawPosPid.loop(yawTarget, yawMotor.getSpeed()));
        }
    }
}

double calculateAngleError(double currAngle, double targetAngle) {
    /* Positive is counter-clockwise */
    return atan2(sin(targetAngle - currAngle), cos(targetAngle - currAngle));
}

double normalizePitchAngle() {
    return -(pitchMotor.getAngle() - degToRad(355.0));
}

void sendGimbMessage(float y) {
    float valScaler = PI / int16_MAX;
    int16_t yawT = static_cast<int16_t>(y / valScaler);
    uint8_t yawT1 = (yawT + int16_MAX) >> 8;
    uint8_t yawT2 = (yawT + int16_MAX);
    int16_t imuY = static_cast<int16_t>(userIMU::imuYaw() / valScaler);
    uint8_t imuY1 = (imuY + int16_MAX) >> 8;
    uint8_t imuY2 = (imuY + int16_MAX);

    // test = static_cast<int16_t>(-PI / valScaler);
    // test1 = (test + int16_MAX) >> 8;
    // test2 = (test + int16_MAX);

    gimbMsg[0] = 'g';
    gimbMsg[1] = static_cast<uint8_t>(currState);
    gimbMsg[2] = yawT1;
    gimbMsg[3] = yawT2;
    gimbMsg[4] = imuY1;
    gimbMsg[5] = imuY2;
    gimbMsg[6] = 'e';
    HAL_UART_Transmit(&huart8, (uint8_t*)gimbMsg, sizeof(gimbMsg), 3);
}

} // namespace gimbal
