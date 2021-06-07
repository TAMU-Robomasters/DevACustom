#include "subsystems/gimbal.hpp"
#include "information/uart_protocol.hpp"
#include "init.hpp"

float32_t bung = 0;
float yawAngleShow, yawErrorShow, yawPidShow, yawDerivShow;
float pitchAngleShow, pitchErrorShow, pitchPidShow, pitchTargetShow;
float currGimbTime, lastGimbTime, lastGimbLoopTime;
float dispYaw;
float dispPitch;
float yawTarget, pitchTarget;

float yawSave = degToRad(270.0);
float pitchSave = degToRad(115.0);

float yawRx;
uint8_t gimbMsg[5];

namespace gimbal {

gimbalStates currState = notRunning;
CtrlTypes ctrlType = VOLTAGE;

filter::Kalman gimbalVelFilter(0.05, 16.0, 1023.0, 0.0);

//pidInstance yawPosPid(pidType::position, 150.0, 0.00, 10000.0);
//pidInstance pitchPosPid(pidType::position, 100.0, 0.0, 2500.0);
pidInstance yawPosPid(pidType::position, 200.0, 0.00, 2.0);
pidInstance pitchPosPid(pidType::position, 100.0, 0.0, 0.5);
float kF = 30;

gimbalMotor yawMotor(userCAN::GM6020_YAW_ID, yawPosPid, gimbalVelFilter);
gimbalMotor pitchMotor(userCAN::GM6020_PIT_ID, pitchPosPid, gimbalVelFilter);

void task() {

    for (;;) {
        update();

        act();
        // set power to global variable here, message is actually sent with the others in the CAN task

        lastGimbTime = currGimbTime;
        currGimbTime = HAL_GetTick();

        lastGimbLoopTime = currGimbTime - lastGimbTime;

        yawPidShow = yawPosPid.getOutput();
        pitchPidShow = pitchPosPid.getOutput();

        yawDerivShow = yawPosPid.getDerivative() * 10000;

        if (operatingType == primary) {
            osDelay(2);
        } else if (operatingType == secondary) {
            osDelay(10);
        } else
            osDelay(5);
    }
}

void update() {

    currState = idle; // default state

    struct userUART::aimMsgStruct* pxAimRxedPointer;
    struct userUART::gimbMsgStruct* pxGimbRxedPointer;

    if (operatingType == primary) {
        currState = idle; // default state

        pitchAngleShow = radToDeg(pitchMotor.getAngle());
        pitchTargetShow = (-(pitchMotor.getAngle() - degToRad(115.0)));
        pitchPidShow = pitchPosPid.getOutput();
        // pitchErrorShow = radToDeg(normalizePitchAngle());

        if (userUART::aimMsgQueue != NULL) {
            if (xQueueReceive(userUART::aimMsgQueue, &(pxAimRxedPointer), (TickType_t)0) == pdPASS) {
                if (pxAimRxedPointer->prefix == userUART::jetsonMsgTypes::aimAt) {
                    dispYaw = pxAimRxedPointer->disp[0];
                    dispPitch = pxAimRxedPointer->disp[1];
                    currState = aimFromCV;
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
								bung++;
                if (pxGimbRxedPointer->prefix == userUART::d2dMsgTypes::gimbal) {
                    currState = pxGimbRxedPointer->state;
                    yawRx = pxGimbRxedPointer->yaw;
                }
            }
        }
    }

    //HAL_UART_Transmit(&huart6, (uint8_t*)"sacke", sizeof("sacke"), 50);

    // if (yawMotor.getAngle() + dispYaw > degToRad(180.0) || yawMotor.getAngle() + dispYaw < degToRad(60.0)) {
    //     currState = idle;
    // }
    // if (pitchMotor.getAngle() + dispPitch > degToRad(180.0) || pitchMotor.getAngle() + dispPitch < degToRad(90.0)) {
    //     currState = idle;
    // }
}

void act() {
    switch (currState) {
    case notRunning:
        // if (operatingType == primary) {
        pitchMotor.setPower(0);
        // }
        // if (operatingType == secondary) {
        yawMotor.setPower(0);
        // }
        break;

    case aimFromCV:
        if (operatingType == primary) {
            pitchSave = pitchMotor.getAngle();
            pitchPosPid.setTarget(0.0);
            pitchTarget = -calculateAngleError(pitchMotor.getAngle(), pitchMotor.getAngle() - dispPitch);
            pitchMotor.setPower(pitchPosPid.loop(pitchTarget, pitchMotor.getSpeed()) + (-kF * cos(normalizePitchAngle())));
            // pitchTarget = -calculateAngleError(pitchMotor.getAngle(), degToRad(115.0));
            // pitchMotor.setPower(-kF * cos(normalizePitchAngle()));
            sendGimbMessage(dispYaw);
        }
        if (operatingType == secondary) {
            yawSave = yawMotor.getAngle();
            yawPosPid.setTarget(0.0);
            yawTarget = -calculateAngleError(yawMotor.getAngle(), yawMotor.getAngle() - yawRx);
            // yawTarget = -calculateAngleError(yawMotor.getAngle(), degToRad(90.0));
            yawMotor.setPower(yawPosPid.loop(yawTarget, yawMotor.getSpeed()));
        }
        break;

    case idle:
        if (operatingType == primary) {
            pitchPosPid.setTarget(0.0);
            pitchTarget = -calculateAngleError(pitchMotor.getAngle(), pitchSave);
            pitchMotor.setPower(pitchPosPid.loop(pitchTarget, pitchMotor.getSpeed()) + (-kF * cos(normalizePitchAngle())));
            // pitchTarget = -calculateAngleError(pitchMotor.getAngle(), degToRad(115.0));
            // pitchMotor.setPower(-kF * cos(normalizePitchAngle()));
            sendGimbMessage();
        }
        if (operatingType == secondary) {
            yawPosPid.setTarget(0.0);
            yawDerivShow = yawMotor.getSpeed() * yawPosPid.getkD();
            yawTarget = -calculateAngleError(yawMotor.getAngle(), yawSave);
            // yawTarget = -calculateAngleError(yawMotor.getAngle(), degToRad(90.0));
            yawMotor.setPower(yawPosPid.loop(yawTarget, yawMotor.getSpeed()));
        }
        break;
    }
}

double normalizePitchAngle() {
    return -(pitchMotor.getAngle() - degToRad(115.0));
}

double calculateAngleError(double currAngle, double targetAngle) {
    /* Positive is counter-clockwise */
    return atan2(sin(targetAngle - currAngle), cos(targetAngle - currAngle));
}

void sendGimbMessage() {
    //uint8_t gimbMsg[5];
    gimbMsg[0] = 'g';
    gimbMsg[1] = static_cast<uint8_t>(currState);
    gimbMsg[2] = 0;
    gimbMsg[3] = 0;
    gimbMsg[4] = 'e';
    HAL_UART_Transmit(&huart8, (uint8_t*)gimbMsg, sizeof(gimbMsg), 1);
}

void sendGimbMessage(float y) {
    int16_t yawT = static_cast<int16_t>(y * 10000);
    uint8_t yawT1 = (yawT + 32768) >> 8;
    uint8_t yawT2 = (yawT + 32768);
    gimbMsg[0] = 'g';
    gimbMsg[1] = static_cast<uint8_t>(currState);
    gimbMsg[2] = yawT1;
    gimbMsg[3] = yawT2;
    gimbMsg[4] = 'e';
    HAL_UART_Transmit(&huart8, (uint8_t*)gimbMsg, sizeof(gimbMsg), 1);
}

} // namespace gimbal
