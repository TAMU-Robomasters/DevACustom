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
double rollSpeed, pitchSpeed, yawSpeed;
int pitchTemp;
double sendTestVal;

int gimbStateShow;

float rcRightX;
float rcRightY;
int keyW;
int keyA;
int keyS;
int keyD;
int mouseX;
int mouseY;

float yawSave = degToRad(94.0);
float pitchSave = degToRad(355.0);
float imuGimbYawSave = degToRad(180.0);
float imuGimbPitchSave = degToRad(0.0);

float yawRx = 0;
float gimbYawIMU = 0;
float gimbYawSpeed = 0;
uint8_t gimbMsg[7];

namespace gimbal {

gimbalStates currState = notRunning;
CtrlTypes ctrlType = VOLTAGE;

filter::Kalman gimbalVelFilter(0.05, 16.0, 1023.0, 0.0);

// pidInstance yawPosPid(pidType::position, 95.0, 0.00, 2100.00);
// pidInstance pitchPosPid(pidType::position, 350.0, 0.00, 6000.0); // not using direct motor speeds
pidInstance yawPosPid(pidType::position, 150.0, 0.0, 0.5);
pidInstance pitchPosPid(pidType::position, 300.0, 0.00, 1.2);
pidInstance yawVelPid(pidType::velocity, 3, 0.0, 0.0);
pidInstance pitchVelPid(pidType::velocity, 7, 0.0, 0.00);

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
    rollSpeed = userIMU::imuRollSpeed();
    pitchSpeed = userIMU::imuPitchSpeed();
    yawSpeed = userIMU::imuYawSpeed();

    if (operatingType == primary) {
        // currState = imuIdle; // default state

        pitchAngleShow = radToDeg(pitchMotor.getAngle());
        pitchTargetShow = radToDeg(pitchPosPid.getTarget());
        pitchPidShow = pitchVelPid.getOutput();
        pitchTemp = pitchMotor.getTemp();
        rcRightX = getJoystick(joystickAxis::rightX);
        rcRightY = getJoystick(joystickAxis::rightY);
        keyW = GET_BIT(rcDataStruct.key.v, btnW);
        keyA = GET_BIT(rcDataStruct.key.v, btnA);
        keyS = GET_BIT(rcDataStruct.key.v, btnS);
        keyD = GET_BIT(rcDataStruct.key.v, btnD);
        mouseX = rcDataStruct.mouse.x;
        mouseY = rcDataStruct.mouse.y;
        float scaled = userIMU::imuYawSpeed() * 100;
        float valScaler = PI / int16_MAX;
        sendTestVal = static_cast<int16_t>((userIMU::imuYawSpeed() / 100) / valScaler);

        currState = imuIdle;
        // if (fabs(getJoystick(joystickAxis::rightX)) > 0 || fabs(getJoystick(joystickAxis::rightY)) > 0) {
        //     currState = rcYawLocked;
        // }
        if (fabs(getMouse(mouseAxis::y)) > 0) {
            currState = rcYawLocked;
        }
        // if (getSwitch(switchType::right) == switchPosition::up) {
        //     currState = rcYawUnlocked;
        // }

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
        // currState = notRunning;

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

    gimbStateShow = currState;

    //if (yawMotor.getAngle() + dispYaw > degToRad(180.0) || yawMotor.getAngle() + dispYaw < degToRad(60.0)) {
    //    currState = idle;
    //}
    //if (pitchMotor.getAngle() + dispPitch > degToRad(180.0) || pitchMotor.getAngle() + dispPitch < degToRad(90.0)) {
    //    currState = idle;
    //}
}

void act() {
    switch (currState) {
    case notRunning: {
        pitchMotor.setPower(0);
        yawMotor.setPower(0);
        if (operatingType == primary) {
            imuGimbPitchSave = userIMU::imuPitch();
            sendGimbMessage(0);
        }
        break;
    }

    case aimFromCV: {
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
    }

    case idle: {
        // yawTarget = -calculateAngleError(yawMotor.getAngle(), yawSave);
        // yawTarget = -calculateAngleError(yawMotor.getAngle(), degToRad(274.0));
        // pitchTarget = -calculateAngleError(pitchMotor.getAngle(), degToRad(21.5));
        // pitchMotor.setPower(-kF * cos(normalizePitchAngle()));
        if (operatingType == primary) {
            imuGimbPitchSave = userIMU::imuPitch();

            pitchPosPid.setTarget(0.0);
            pitchTarget = -calculateAngleError(pitchMotor.getAngle(), pitchSave);
            pitchMotor.setPower(pitchPosPid.loop(pitchTarget, pitchMotor.getSpeed()) + (-kF * cos(normalizePitchAngle())));
            // pitchMotor.setPower(-kF * cos(normalizePitchAngle()));
            sendGimbMessage(0);
        }
        if (operatingType == secondary) {
            imuGimbYawSave = gimbYawIMU;

            yawPosPid.setTarget(0.0);
            yawDerivShow = yawMotor.getSpeed() * yawPosPid.getkD();
            yawTarget = -calculateAngleError(yawMotor.getAngle(), yawSave);
            // yawMotor.setPower(yawPosPid.loop(yawTarget, yawMotor.getSpeed()));
        }
        break;
    }

    case imuIdle: {
        if (operatingType == primary) {
            pitchVelPid.setTarget(0.0);
            pitchMotor.setPower(-pitchVelPid.loop(userIMU::imuPitchSpeed()) + (-kF * cos(normalizePitchAngle())));
            sendGimbMessage(0, userIMU::imuYawSpeed() / 100.0f);
        }
        if (operatingType == secondary) {
            yawPosPid.setTarget(0.0);
            yawDerivShow = yawMotor.getSpeed() * yawPosPid.getkD();
            yawTarget = -calculateAngleError(yawMotor.getAngle(), degToRad(94.0));
            yawMotor.setPower(yawPosPid.loop(yawTarget, yawMotor.getSpeed()));
        }
        break;
    }

    case rcYawLocked: {
        if (operatingType == primary) {
            // dispPitch = getJoystick(joystickAxis::rightY) * 15.0f;
            dispPitch = getMouse(mouseAxis::y) * 0.1f;
            pitchVelPid.setTarget(dispPitch);
            pitchMotor.setPower(-pitchVelPid.loop(userIMU::imuPitchSpeed()) + (-kF * cos(normalizePitchAngle())));
            sendGimbMessage(0, userIMU::imuYawSpeed() / 100.0f);
        }
        if (operatingType == secondary) {
            yawPosPid.setTarget(0.0);
            yawDerivShow = yawMotor.getSpeed() * yawPosPid.getkD();
            yawTarget = -calculateAngleError(yawMotor.getAngle(), degToRad(94.0));
            yawMotor.setPower(yawPosPid.loop(yawTarget, yawMotor.getSpeed()));
        }
        break;
    }

    case rcYawUnlocked: {
        if (operatingType == primary) {
            dispPitch = getJoystick(joystickAxis::rightY) * 15.0f;
            pitchVelPid.setTarget(-dispPitch);
            pitchMotor.setPower(-pitchVelPid.loop(userIMU::imuPitchSpeed()) + (-kF * cos(normalizePitchAngle())));
            sendGimbMessage(0, userIMU::imuYawSpeed() / 100.0f);
        }
        if (operatingType == secondary) {
            dispYaw = getJoystick(joystickAxis::rightX) * 15.0f;
            yawVelPid.setTarget(-dispYaw);
            yawDerivShow = yawMotor.getSpeed() * yawVelPid.getkD();
            yawMotor.setPower(yawVelPid.loop(gimbYawIMU * 100));
        }
        break;
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

void sendGimbMessage(float y, float imuData) {
    float valScaler = PI / int16_MAX;
    int16_t yawT = static_cast<int16_t>(y / valScaler);
    uint8_t yawT1 = (yawT + int16_MAX) >> 8;
    uint8_t yawT2 = (yawT + int16_MAX);
    int16_t imuY = static_cast<int16_t>(imuData / valScaler);
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
