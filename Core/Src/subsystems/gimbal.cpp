#include "subsystems/gimbal.hpp"
#include "init.hpp"

namespace gimbal {

gimbalStates currState = notRunning;
CtrlTypes ctrlType = VOLTAGE;

pidInstance yawPosPid(pidInstance::type::position, 3.0, 0.0, 0.0);
pidInstance pitchPosPid(pidInstance::type::position, 0.0, 0.0, 0.0);

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

    if (true) {
        currState = running;
        // will change later based on RC input and sensor based decision making
    }

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
            double power = 0;
            //double power = yawPosPid.loop(input);
            yawMotor.setPower(power);
            pitchMotor.setPower(power);
        }
        // obviously this will change when we have actual intelligent things to put here
        break;
    }
}

double calculateAngularPath(double currAngle, double targetAngle) {
	//Positive is counterclockwise while Negative is clockwise
    double angleDelta = currAngle - targetAngle;
	
	if (fabs(angleDelta <= PI)) //Check if within simple case of within 180 degrees
	{
		return -angleDelta;
	}
	else if(angleDelta > PI)
	{
		return targetAngle - 2*PI + currAngle; //Removes the need to do the negative operator since clockwise
	}
	else //angleDelta < -PI
	{
		return currAngle + 2*PI - targetAngle;
	}
}

} // namespace gimbal
