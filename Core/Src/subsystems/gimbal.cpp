#include "subsystems/gimbal.hpp"
#include "init.hpp"

namespace gimbal {

gimbalStates currState = notRunning;
CtrlTypes ctrlType = VOLTAGE;

pidInstance yawPosPid(pidType::position, 3.0, 0.0, 0.0);
pidInstance pitchPosPid(pidType::position, 0.0, 0.0, 0.0);

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
    yawMotor.setCurrAngle(static_cast<double>(yawMotor.getFeedback()->rotor_angle));
    pitchMotor.setCurrAngle(static_cast<double>(pitchMotor.getFeedback()->rotor_angle));

    int t = 0;
    bool looped = false;
    if (t < 1000 && looped == false) {      //Arbitrary number of 2 seconds for loops   
        currState = patrol;
        t++;
    }
    else {
        currState = notRunning;
        looped = true;
        t--;
    }
    if (t == 0) {
        looped = false;
    }
    // Update encoders

    yawPosPid.setTarget(0);
    pitchPosPid.setTarget(0);
    // if button pressed on controller, change state to "followgimbal" or something
}

void act() {
    switch (currState) {
    case notRunning:
        yawMotor.setPower(0);
        pitchMotor.setPower(0);
        break;

    case running:
        if (ctrlType == VOLTAGE) {
            double power = yawPosPid.loop(calculateAngleError(yawMotor.getCurrAngle(), yawPosPid.getTarget()));
            yawMotor.setPower(power);
            pitchMotor.setPower(power);
        }
        // obviously this will change when we have actual intelligent things to put here
        break;
    case patrol: 
        bool patrolLoopYaw = false;
        bool patrolLoopPitch = false;
        if (180 - static_cast<double>(yawMotor.getFeedback()->rotor_angle < .1) {     //FIXME: is yaw or pitch 180, TOL
            patrolLoopYaw = true;
        }
        else if (static_cast<double>(yawMotor.getFeedback()->rotor_angle < .1) {  //FIXME: ENCODERS       if (encoders == 0) within a tolerance
            patrolLoopYaw = false;
        }
        if (180 - static_cast<double>(pitchMotor.getFeedback()->rotor_angle < .1) {     //FIXME: is yaw or pitch 180, TOL
            patrolLoopPitch = true;
        }
        else if (static_cast<double>(pitchMotor.getFeedback()->rotor_angle < .1) {  //FIXME: ENCODERS       if (encoders == 0) within a tolerance
            patrolLoopPitch = false;
        }
        if (patrolLoopYaw == false && patrolLoopPitch == false) {
            if (ctrlType == VOLTAGE) {
                double power = yawPosPid.loop(calculateAngleError(yawMotor.getCurrAngle(), yawPosPid.getTarget()));
                yawMotor.setPower(power);
                pitchMotor.setPower(power);
            }
        }
        if (patrolLoopYaw == false && patrolLoopPitch == true) {
            if (ctrlType == VOLTAGE) {
                double power = yawPosPid.loop(calculateAngleError(yawMotor.getCurrAngle(), yawPosPid.getTarget()));
                yawMotor.setPower(power);
                pitchMotor.setPower(-power);
            }
        }
        if (patrolLoopYaw == true && patrolLoopPitch == false) {
            if (ctrlType == VOLTAGE) {
                double power = yawPosPid.loop(calculateAngleError(yawMotor.getCurrAngle(), yawPosPid.getTarget()));
                yawMotor.setPower(-power);
                pitchMotor.setPower(power);
            }
        }
        if (patrolLoop == true && patrolLoopPitch == true) {
            if (ctrlType == VOLTAGE) {
                double power = -yawPosPid.loop(calculateAngleError(yawMotor.getCurrAngle(), yawPosPid.getTarget()));
                yawMotor.setPower(power);
                pitchMotor.setPower(power);
            }
        }
        break;
    }
}

double fixAngle(double angle) {
    /* Fix angle to [0, 2PI) */
    angle = fmod(angle, 2 * PI);
    if (angle < 0)
        return angle + 2 * PI;
    return angle;
}

double calculateAngleError(double currAngle, double targetAngle) {
    /* Positive is counter-clockwise */

    double angleDelta = fixAngle(targetAngle) - fixAngle(currAngle);
	
	if (fabs(angleDelta) <= PI)
	{
		return angleDelta;
	}
	else if(angleDelta > PI)
	{
		return -(2 * PI - angleDelta);
	}
	else // angleDelta < -PI
	{
		return (2 * PI + angleDelta);
	}
}

} // namespace gimbal
