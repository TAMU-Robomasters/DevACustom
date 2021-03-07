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

// pidInstance yawPosPid(pidType::position, 70.0, 0.00, 0.01);
pidInstance yawPosPid(pidType::position, 80.0, 0.00, 0.00);
pidInstance pitchPosPid(pidType::position, 140.0, 0.0, 0.01);

gimbalMotor yawMotor(userCAN::GM6020_YAW_ID, yawPosPid);
gimbalMotor pitchMotor(userCAN::GM6020_PIT_ID, pitchPosPid);

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
        currState = notRunning;
        // will change later based on RC input and sensor based decision making
    }

    float yawAngle = yawMotor.getAngle();
    float pitchAngle = pitchMotor.getAngle();
    yawAngleShow = radToDeg(yawAngle);
		pitchAngleShow = radToDeg(pitchAngle);

        float yawTarget = degToRad(207.0);
        float pitchTarget = degToRad(309.0);
        float yawError = calculateAngleError(yawAngle, yawTarget);
		float pitchError = calculateAngleError(pitchAngle, pitchTarget);
		
		yawErrorShow = radToDeg(yawError);
        pitchErrorShow = radToDeg(pitchError);
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
                    yawMotor.setPower(yawPosPid.loop(-calculateAngleError(yawMotor.getAngle(), degToRad(207.0))));
                    pitchMotor.setPower(pitchPosPid.loop(-calculateAngleError(pitchMotor.getAngle(), degToRad(309.0))));
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
