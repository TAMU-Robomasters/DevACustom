#include "subsystems/gimbal.hpp"
#include "information/uart_protocol.hpp"
#include "init.hpp"

float32_t bung = 0;
float yawAngleShow;
float yawErrorShow;
float yawPidShow;
float pitchAngleShow;
float pitchErrorShow;
float pitchPidShow;
float currGimbTime;
float lastGimbTime;
float lastGimbLoopTime;
float pitchPowerShow;
float dispYaw, dispPitch;
float yawSave = 274.0*PI/180;
float pitchSave = 16*PI/180;

namespace gimbal {

gimbalStates currState = notRunning;
CtrlTypes ctrlType = VOLTAGE;
	
filter::Kalman gimbalVelFilter(0.05, 16.0, 1023.0, 0.0);

pidInstance yawPosPid(pidType::position, 50.0, 0.00, 0.00);
pidInstance pitchPosPid(pidType::position, 220.0, 0.00, 1.0);
float kF = 0;

gimbalMotor yawMotor(userCAN::GM6020_YAW_ID, yawPosPid, gimbalVelFilter);
gimbalMotor pitchMotor(userCAN::GM6020_PIT_ID, pitchPosPid, gimbalVelFilter);

void task() {

    for (;;) {
        update();

        act();
        // set power to global variable here, message is actually sent with the others in the CAN task

        osDelay(10);
    }
}

void update() {

    currState = idle; // default state

    struct userUART::gimbMessage* pxRxedPointer;

    if (xQueueReceive(userUART::gimbalQueue, &(pxRxedPointer), (TickType_t)0) == pdPASS) {
        if (pxRxedPointer->prefix == userUART::msgTypes::aimAt) {
            dispYaw = pxRxedPointer->disp[0];
            dispPitch = pxRxedPointer->disp[1];
            currState = aimFromCV;
        }
    }

    //if (yawMotor.getAngle() + dispYaw > degToRad(180.0) || yawMotor.getAngle() + dispYaw < degToRad(60.0)) {
    //    currState = idle;
    //}
    //if (pitchMotor.getAngle() + dispPitch > degToRad(180.0) || pitchMotor.getAngle() + dispPitch < degToRad(90.0)) {
    //    currState = idle;
    //}

    float yawAngle = yawMotor.getAngle();
    float pitchAngle = pitchMotor.getAngle();
    yawAngleShow = radToDeg(yawAngle);
    pitchAngleShow = radToDeg(pitchAngle);

    pitchErrorShow = (-(pitchMotor.getAngle() - degToRad(310.0)));
    // if button pressed on controller, change state to "followgimbal" or something
}

void act() {
    switch (currState) {
    case notRunning:
        yawMotor.setPower(0);
        pitchMotor.setPower(0);
        break;

    case aimFromCV:
        yawPosPid.setTarget(0.0);
        pitchPosPid.setTarget(0.0);

        yawPidShow = yawPosPid.getOutput();
        pitchPidShow = pitchPosPid.getOutput();
		
				yawSave = yawMotor.getAngle();
				pitchSave = pitchMotor.getAngle();

        if (ctrlType == VOLTAGE) { // gimbal motors controlled through voltage, sent messages over CAN
            double yawError = -calculateAngleError(yawMotor.getAngle(), yawMotor.getAngle() - dispYaw);
            //double yawError = -calculateAngleError(yawMotor.getAngle(), degToRad(274.0));
            double pitchError = -calculateAngleError(pitchMotor.getAngle(), pitchMotor.getAngle() - dispPitch);
            //double pitchError = -calculateAngleError(pitchMotor.getAngle(), degToRad(21.5));
            yawMotor.setPower(yawPosPid.loop(yawError));
            // pitchMotor.setPower(-kF * cos(normalizePitchAngle()));
            pitchMotor.setPower(pitchPosPid.loop(pitchError) + (-kF * cos(normalizePitchAngle())));
        }
        break;

    case idle:
        yawPosPid.setTarget(0.0);
        pitchPosPid.setTarget(0.0);

        yawPidShow = yawPosPid.getOutput();
        pitchPidShow = pitchPosPid.getOutput();

        if (ctrlType == VOLTAGE) { // gimbal motors controlled through voltage, sent messages over CAN
            double yawError = -calculateAngleError(yawMotor.getAngle(), yawSave);
            //double yawError = -calculateAngleError(yawMotor.getAngle(), degToRad(274.0));
            double pitchError = -calculateAngleError(pitchMotor.getAngle(), pitchSave);
            //double pitchError = -calculateAngleError(pitchMotor.getAngle(), degToRad(21.5));
            yawMotor.setPower(yawPosPid.loop(yawError));
            //pitchMotor.setPower(-kF * cos(normalizePitchAngle()));
            pitchMotor.setPower(pitchPosPid.loop(pitchError) + (-kF * cos(normalizePitchAngle())));
        }
        break;
    }
}

double fixAngle(double angle) {
    /* Fix angle to [0, 2PI) */
    angle = fmod(angle, 2 * PI);
    if (angle < 0){
        return angle + 2 * PI;
		}
    return angle;
}

double calculateAngleError(double currAngle, double targetAngle) {
  /* Positive is counter-clockwise */

	return atan2(sin(targetAngle-currAngle), cos(targetAngle-currAngle));

    // double angleDelta = fixAngle(targetAngle) - fixAngle(currAngle);

	// if (fabs(angleDelta) <= PI)
	// {
	// 	return angleDelta;
	// }
	// else if(angleDelta > PI)
	// {
	// 	return -(2 * PI - angleDelta);
	// }
	// else // angleDelta < -PI
	// {
	// 	return (2 * PI + angleDelta);
	// }

  // return 69;
}

double normalizePitchAngle(){
		return -(pitchMotor.getAngle() - degToRad(21.5));
}

} // namespace gimbal
