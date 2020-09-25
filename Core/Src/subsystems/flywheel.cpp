#include "subsystems/flywheel.hpp"
#include "init.hpp"

namespace flywheel {

flywheelStates currState = notRunning;

double angularAccLimit = 0.1; // defined as rpm/10ms^2

flywheelMotor flyhweel1(&htim2, 1, POWER1_CTRL_GPIO_Port, POWER1_CTRL_Pin);

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
}

void act() {
    switch (currState) {
    case notRunning:
        break;

    case running:
        // obviously this will change when we have actual intelligent things to put here
        break;
    }
}

double calcSlewDRpm(double currFw1Speed, double currFw2Speed, double targetFw1Speed, double targetFw2Speed) {
    // IMPLEMENT THIS
    /* 
    Given the current speeds (rpm) of both flywheel motors, a target speed, and an angular acceleration cap
    (globally defined as angularAccLimit on line 8)
        - Calculate the largest amount the flywheel speed should change by while still
        adhering to the acceleration constraint.
        - For now, set your calculated values to fw1DRPM and fw2DRPM.

    EX: If the current speed of the flywheels are 50rpm, and the target speed given is 80rpm, calculate what
    speed the flywheels should be set to THIS LOOP while adhering to the acceleration limit.

    NOTE: As I write this, I'm realizing this task might be dummy simple. If that's the case, go ahead and have
    some fun with it, whether that means adding some more kinematic constraints, or allowing us to set a current
    cap rather than an acceleration cap; idc the sky's the limit. I'll get better at writing these as time goes :/
    */

    double fw1DRPM;
    double fw2DRPM;
    return 0;
}

} // namespace flywheel
