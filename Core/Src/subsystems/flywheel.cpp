#include "subsystems/flywheel.hpp"
#include "arm_math.h"
#include "init.hpp"

namespace flywheel {

flywheelStates currState = notRunning;

double angularAccLimit = 0.1; // defined as rpm/10ms^2

flywheelMotor flywheel1(&htim2, 1, POWER1_CTRL_GPIO_Port, POWER1_CTRL_Pin);
flywheelMotor flywheel2(&htim2, 2, POWER3_CTRL_GPIO_Port, POWER3_CTRL_Pin);

void task() {

    flywheel1.initESC();
    flywheel2.initESC();

    for (;;) {
        update();

        act();
        // set power to global variable here, message is actually sent with the others in the CAN task

        osDelay(10);
    }
}

void update() {
    // If flywheel switch is on turn on flywheel
    if (true) {         //FIXME change to switch input
        currState = running;
    }
    else {
        currState = notRunning;
    }
}

void act() {
    switch (currState) {
    case notRunning:
        flywheel1.setPower(0);
        flywheel2.setPower(0);
        break;

    case running:
        calcSlewDRpm(flywheel1.getPower(), flywheel2.getPower(), 10, 10);
        // obviously this will change when we have actual intelligent things to put here
        break;
    }
}

double calcSlewDRpm(double currFw1Speed, double currFw2Speed, double targetFw1Speed, double targetFw2Speed) {
    double fw1DRPM;
    double fw2DRPM;
    double velLim = angularAccLimit * 10;

    if(angularAccLimit > abs(targetFw1Speed - currFw1Speed) / 10) {
        fw1DRPM = targetFw1Speed - currFw1Speed;
    }
    else if(angularAccLimit < abs(targetFw1Speed - currFw1Speed) / 10) {
        if(targetFw1Speed - currFw1Speed < 0) {
            fw1DRPM = -velLim;
        }
        else if(targetFw1Speed - currFw1Speed > 0) {
            fw1DRPM = velLim;
        }
    }
    else {
        fw1DRPM = 0;
    }

    if(angularAccLimit > abs(targetFw2Speed - currFw2Speed) / 10) {
        fw2DRPM = targetFw2Speed - currFw2Speed;
    }
    else if(angularAccLimit < abs(targetFw2Speed - currFw2Speed) / 10) {
        if(targetFw2Speed - currFw2Speed < 0) {
            fw2DRPM = -velLim;
        }
        else if(targetFw2Speed - currFw2Speed > 0) {
            fw2DRPM = velLim;
        }
    }
    else {
        fw2DRPM = 0;
    }

    flywheel1.setPower(fw1DRPM);
    flywheel2.setPower(fw2DRPM);

    return 0;
}

} // namespace flywheel
