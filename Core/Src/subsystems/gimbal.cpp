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

double calculateAngleError(double currAngle, double targetAngle) {
    // IMPLEMENT THIS
    /* 
    Given a current angle value (currAngle, in radians) and a target angle value (targetAngle, in radians):
        - Find and return the shortest angle error in radians with direction.
    (hint hint use trig functions)

    EX: If our current angle is 359 degrees, and the target angle is 2 degrees, this function
        should return 3 degrees, as the shortest path from the current angle to the target angle
        will be 3 degrees clockwise.
        (the example was in degrees, please use radians)
    */
    return 0;
}

} // namespace gimbal
