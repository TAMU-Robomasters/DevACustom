#include "information/uart_protocol.hpp"
#include "subsystems/gimbal.hpp"
#include "init.hpp"

float32_t bung = 0;
float yawAngleShow;
float yawErrorShow;
float yawPidShow;
float pitchAngleShow;
float pitchErrorShow;
float pitchPidShow;

namespace gimbal {
	
gimbalStates currState = notRunning;
CtrlTypes ctrlType = VOLTAGE;

filter::Kalman gimbalVelFilter(0.05, 16.0, 1023.0, 0.0);

pidInstance yawPosPid(pidType::position, 20.0, 0.00, 0.00);
pidInstance pitchPosPid(pidType::position, 40.0, 0.0, 0.00);
//pidInstance yawPosPid(pidType::position, 20.0, 0.00, 0.00);
//pidInstance pitchPosPid(pidType::position, 20.0, 0.0, 0.00);
float kF = 25;

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
    if (true) {
        currState = running;
        // will change later based on RC input and sensor based decision making
    }

    float yawAngle = yawMotor.getAngle();
    float pitchAngle = pitchMotor.getAngle();
    yawAngleShow = radToDeg(yawAngle);
		pitchAngleShow = radToDeg(pitchAngle);

    //float yawTarget = degToRad(85.0);
		float yawTarget = angleX;
    //float pitchTarget = degToRad(260.0);
		float pitchTarget = angleY;
    float yawError = calculateAngleError(yawAngle, yawTarget);
		float pitchError = calculateAngleError(pitchAngle, pitchTarget);
		
		yawErrorShow = radToDeg(yawError);
    pitchErrorShow = radToDeg(normalizePitchAngle());
    // if button pressed on controller, change state to "followgimbal" or something
}

void act() {
    switch (currState) {
    case notRunning:
        yawMotor.setPower(0);
        pitchMotor.setPower(0);
        break;

    case running:
				yawPosPid.setTarget(0.0);
        pitchPosPid.setTarget(0.0);
				// float yawPower = yawPosPid.getOutput();
				// float pitchPower = pitchPosPid.getOutput();
				if (ctrlType == VOLTAGE) {
					yawPidShow = yawPosPid.getOutput();
					pitchPidShow = pitchPosPid.getOutput();
					double yawError = -calculateAngleError(yawMotor.getAngle(), degToRad(90.0));
					double pitchError = -calculateAngleError(pitchMotor.getAngle(), degToRad(115.0));
					//yawMotor.setPower(yawPosPid.loop(yawError));
					//pitchMotor.setPower(-kF * cos(normalizePitchAngle()));
					pitchMotor.setPower(pitchPosPid.loop(pitchError) + (-kF * cos(normalizePitchAngle())));
        } // gimbal motors controlled through voltage, sent messages over CAN
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

double normalizePitchAngle(){
		return -(pitchMotor.getAngle() - degToRad(115.0));
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

} // namespace gimbal
