#include "subsystems/gimbal.hpp"
#include "imu/imu_protocol.hpp"
#include "information/uart_protocol.hpp"
#include "init.hpp"
#include "subsystems/feeder.hpp"
#include "subsystems/flywheel.hpp"

float32_t bung = 0;
float yawAngleShow, yawErrorShow, yawPidShow, yawDerivShow;
float pitchAngleShow, pitchErrorShow, pitchPidShow, pitchTargetShow;
float currGimbTime, lastGimbTime, lastGimbLoopTime;
int pitchTemp, yawTemp;

float dispYaw;
float dispPitch;
float xStddev;
float yStddev;

float yawTarget, pitchTarget;
float roll, pitch, yaw;
int gimbStateShow;

float yawSave = degToRad(180.0);
float pitchSave = degToRad(270.0);

float yawRx;
uint8_t gimbMsg[5];
uint8_t jetsonMsgOut[4];

int startTime;
int messagesPerSec;

float jetsonMessageTimeout = 0;
float dev2devGimbTimeout = 0;

namespace gimbal {

gimbalStates currState = originalIdle;
CtrlTypes ctrlType = VOLTAGE;

filter::Kalman gimbalVelFilter(0.05, 16.0, 1023.0, 0.0);

pidInstance pitchPosPid(pidType::position, 75.0, 0.001, 1.3);
pidInstance yawPosPid(pidType::position, 150.0, 0.00, 2.5);
// pidInstance pitchPosPid(pidType::position, 100.0, 0.001, 0.7);
float kF = 40;

gimbalMotor yawMotor(userCAN::GM6020_YAW_ID, yawPosPid, gimbalVelFilter);
gimbalMotor pitchMotor(userCAN::GM6020_PIT_ID, pitchPosPid, gimbalVelFilter);

void task() {

    startTime = HAL_GetTick();

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

    struct userUART::aimMsgStruct* pxAimRxedPointer;
    struct userUART::gimbMsgStruct* pxGimbRxedPointer;

    if (operatingType == primary) {
        // currState = idle; // default state
        messagesPerSec = bung / ((HAL_GetTick() - startTime) / 1000);

        pitchAngleShow = radToDeg(pitchMotor.getAngle());
        pitchTargetShow = normalizePitchAngle();
        pitchPidShow = pitchPosPid.getOutput();
        pitchErrorShow = calculateAngleError(pitchMotor.getAngle(), pitchPosPid.getTarget());
        pitchTemp = pitchMotor.getTemp();

        userIMU::imuUpdate();

        roll = userIMU::imuRoll();
        pitch = userIMU::imuPitch();
        yaw = userIMU::imuYaw();

        if (jetsonMessageTimeout > 3000 || yStddev == 1) {
						dispYaw = 0;
						dispPitch = 0;
            currState = originalIdle;
						// flywheel::currState = flywheel::flywheelStates::notRunning;
            feeder::currState = feeder::feederStates::notRunning;
        }

        if (userUART::aimMsgQueue != NULL) {
            if (xQueueReceive(userUART::aimMsgQueue, &(pxAimRxedPointer), (TickType_t)0) == pdPASS) {
                if (pxAimRxedPointer->prefix == userUART::jetsonMsgTypes::aimAt) {
                    bung++;
                    dispYaw = pxAimRxedPointer->disp[0];
                    dispPitch = pxAimRxedPointer->disp[1];
                    xStddev = pxAimRxedPointer->stddev[0];
                    yStddev = pxAimRxedPointer->stddev[1];
                    currState = aimFromCV;
                    if (xStddev == 1) {
												//if (feeder::currState == feeder::feederStates::notRunning) {
												//}
                        // flywheel::currState = flywheel::flywheelStates::running;
                        feeder::currState = feeder::feederStates::running;
                    } else {
                        // flywheel::currState = flywheel::flywheelStates::notRunning;
                        feeder::currState = feeder::feederStates::notRunning;
                    }
										//if (yStddev == 1) {
										//		flywheel::currState = flywheel::flywheelStates::notRunning;
                    //    feeder::currState = feeder::feederStates::notRunning;
										//		currState = originalIdle;
										//}
                    jetsonMessageTimeout = 0;
                }
            }
        }
        // sendJetsonMessage(normalizePitchAngle());
        gimbStateShow = currState;
				jetsonMessageTimeout += 2;
    }

    if (operatingType == secondary) {
        //currState = notRunning;

        yawAngleShow = radToDeg(yawMotor.getAngle());
        yawPidShow = yawPosPid.getOutput();

        userIMU::imuUpdate();

        roll = userIMU::imuRoll();
        pitch = userIMU::imuPitch();
        yaw = userIMU::imuYaw();
			
				if (dev2devGimbTimeout > 50){
						currState = notRunning;
						yawRx = 0;
				}

        if (userUART::gimbMsgQueue != NULL) {
            if (xQueueReceive(userUART::gimbMsgQueue, &(pxGimbRxedPointer), (TickType_t)0) == pdPASS) {
                if (pxGimbRxedPointer->prefix == userUART::d2dMsgTypes::gimbal) {
                    bung++;
                    currState = pxGimbRxedPointer->state;
                    yawRx = pxGimbRxedPointer->yaw;

                }
            }
        }
        gimbStateShow = currState;
				dev2devGimbTimeout += 10;
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
    case notRunning: {
        // if (operatingType == primary) {
        pitchMotor.setPower(0);
        // }
        // if (operatingType == secondary) {
        yawMotor.setPower(0);
        // }
        sendGimbMessage(0);
        break;
		}

    case aimFromCV: {
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
            yawTarget = -calculateAngleError(yawMotor.getAngle(), yawMotor.getAngle() + yawRx);
            // yawTarget = -calculateAngleError(yawMotor.getAngle(), degToRad(90.0));
            yawMotor.setPower(yawPosPid.loop(yawTarget, yawMotor.getSpeed()));
        }
        break;
			}

    case idle: {
        if (operatingType == primary) {
            pitchPosPid.setTarget(0.0);
            pitchTarget = -calculateAngleError(pitchMotor.getAngle(), pitchSave);
            pitchMotor.setPower(pitchPosPid.loop(pitchTarget, pitchMotor.getSpeed()) + (-kF * cos(normalizePitchAngle())));
            // pitchTarget = -calculateAngleError(pitchMotor.getAngle(), degToRad(115.0));
            // pitchMotor.setPower(-kF * cos(normalizePitchAngle()));
            sendGimbMessage(0);
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

    case originalIdle: {
        if (operatingType == primary) {
            pitchPosPid.setTarget(0.0);
            pitchTarget = -calculateAngleError(pitchMotor.getAngle(), degToRad(305.0));
            pitchMotor.setPower(pitchPosPid.loop(pitchTarget, pitchMotor.getSpeed()) + (-kF * cos(normalizePitchAngle())));
            sendGimbMessage(0);
        }
        if (operatingType == secondary) {
            //yawPosPid.setTarget(0.0);
            //yawDerivShow = yawMotor.getSpeed() * yawPosPid.getkD();
            //yawTarget = -calculateAngleError(yawMotor.getAngle(), degToRad(220.0));
            //yawMotor.setPower(yawPosPid.loop(yawTarget, yawMotor.getSpeed()));
						
						//yawSave = yawMotor.getAngle();
            //yawPosPid.setTarget(0.0);
            //yawTarget = -calculateAngleError(yawMotor.getAngle(), yawMotor.getAngle() + 0.1f);
						//yawPidShow = yawPosPid.getOutput();
            //yawMotor.setPower(yawPosPid.loop(yawTarget, yawMotor.getSpeed()));
						yawMotor.setPower(7);
        }
        break;
			}
    }
}

double normalizePitchAngle() {
    return -(pitchMotor.getAngle() - degToRad(267.0));
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

void sendJetsonMessage(float p) {
    int16_t pitchT = static_cast<int16_t>(p * 10000);
    uint8_t pitchT1 = (pitchT + 32768) >> 8;
    uint8_t pitchT2 = (pitchT + 32768);
    jetsonMsgOut[0] = 'p';
    jetsonMsgOut[1] = pitchT1;
    jetsonMsgOut[2] = pitchT2;
    jetsonMsgOut[3] = 'e';
    HAL_UART_Transmit(&huart7, (uint8_t*)jetsonMsgOut, sizeof(jetsonMsgOut), 1);
}

} // namespace gimbal
