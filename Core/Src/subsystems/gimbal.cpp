#include "subsystems/gimbal.hpp"
#include "imu/imu_protocol.hpp"
#include "information/uart_protocol.hpp"
#include "information/rc_protocol.h"
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
float currGimbTime;
float lastGimbTime;
float lastGimbLoopTime;
float pitchPowerShow;
float dispYaw;
float dispPitch;
float roll, pitch, yaw;

float rcRightX;

float yawSave = degToRad(94.0);
float pitchSave = degToRad(355.0);

float yawRx;
uint8_t gimbMsg[5];

namespace gimbal {

gimbalStates currState = notRunning;
CtrlTypes ctrlType = VOLTAGE;
	
filter::Kalman gimbalVelFilter(0.05, 16.0, 1023.0, 0.0);

// pidInstance yawPosPid(pidType::position, 95.0, 0.00, 2100.00);
// pidInstance pitchPosPid(pidType::position, 350.0, 0.00, 6000.0); // not using direct motor speeds
pidInstance yawPosPid(pidType::position, 100.0, 0.00, 0.6);
pidInstance pitchPosPid(pidType::position, 350.0, 0.00, 1.0);
float kF = 0;

gimbalMotor yawMotor(userCAN::GM6020_YAW_ID, yawPosPid, gimbalVelFilter);
gimbalMotor pitchMotor(userCAN::GM6020_PIT_ID, pitchPosPid, gimbalVelFilter);

void task() {

    for (;;) {
        update();

        act();
        // set power to global variable here, message is actually sent with the others in the CAN task
				
        if (operatingType == primary){
            osDelay(2);
        }
        else if (operatingType == secondary){
            osDelay(10);
        }
        else
            osDelay(5);
    }
}

void update() {
    //currState = notRunning;
    struct userUART::aimMsgStruct* pxAimRxedPointer;
    struct userUART::gimbMsgStruct* pxGimbRxedPointer;

    if (operatingType == primary) {
        currState = idle; // default state

        pitchAngleShow = radToDeg(pitchMotor.getAngle());
        pitchTargetShow = (-(pitchMotor.getAngle() - degToRad(310.0)));
        pitchPidShow = pitchPosPid.getOutput();

        userIMU::imuUpdate();

        //roll = userIMU::imuRoll();
        //pitch = userIMU::imuPitch();
        //yaw = userIMU::imuYaw();

        roll = imu.mx;
        pitch = imu.my;
        yaw = imu.mz;

        if (/*fabs(getJoystick(joystickAxis::rightX)) >= 0.05f || */ fabs(getJoystick(joystickAxis::rightY)) >= 0.05f) {
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
                    yawRx = pxGimbRxedPointer->yaw;
                }
            }
        }
    }

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
        // if (operatingType == primary) {
        pitchMotor.setPower(0);
        // }
        // if (operatingType == secondary) {
        yawMotor.setPower(0);
        // }
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
            pitchPosPid.setTarget(0.0);
            pitchTarget = -calculateAngleError(pitchMotor.getAngle(), pitchMotor.getAngle() - dispPitch);
            pitchMotor.setPower(pitchPosPid.loop(pitchTarget, pitchMotor.getSpeed()) + (-kF * cos(normalizePitchAngle())));
            sendGimbMessage(dispYaw);
        }
        if (operatingType == secondary) {
            yawSave = yawMotor.getAngle();
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
            pitchPosPid.setTarget(0.0);
            pitchTarget = -calculateAngleError(pitchMotor.getAngle(), pitchSave);
            pitchMotor.setPower(pitchPosPid.loop(pitchTarget, pitchMotor.getSpeed()) + (-kF * cos(normalizePitchAngle())));
            sendGimbMessage();
        }
        if (operatingType == secondary) {
            yawPosPid.setTarget(0.0);
            yawDerivShow = yawMotor.getSpeed() * yawPosPid.getkD();
            yawTarget = -calculateAngleError(yawMotor.getAngle(), yawSave);
            yawMotor.setPower(yawPosPid.loop(yawTarget, yawMotor.getSpeed()));
        }
        break;

    case rc:
        if (operatingType == primary) {
            rcRightX = getJoystick(joystickAxis::rightX);
            if (fabs(getJoystick(joystickAxis::rightY)) >= 0.05f) {
                pitchSave = pitchMotor.getAngle();
                pitchPosPid.setTarget(0.0);
                dispPitch = -getJoystick(joystickAxis::rightY) / 10.0f + 0.03f /*+ std::copysign(0.02f, -getJoystick(joystickAxis::rightY))*/;

                pitchTarget = -calculateAngleError(pitchMotor.getAngle(), pitchMotor.getAngle() - dispPitch);
                pitchMotor.setPower(pitchPosPid.loop(pitchTarget, pitchMotor.getSpeed()) + (-kF * cos(normalizePitchAngle())));
            }
            sendGimbMessage();
            // if (getJoystick(joystickAxis::rightX) != 0){
                // dispYaw = -getJoystick(joystickAxis::rightX) / 6.0f;
                // sendGimbMessage(dispYaw);
            // }
        }
        if (operatingType == secondary) {
            yawPosPid.setTarget(0.0);
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
