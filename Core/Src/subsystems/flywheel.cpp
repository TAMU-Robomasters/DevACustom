#include "subsystems/flywheel.hpp"
#include "init.hpp"

namespace flywheel {

flywheelStates currState = notRunning;

float f1Power = 0;
float f2Power = 0;

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
        f1Power = f2Power = 0;
        break;

    case running:
        f1Power = f2Power = 0;
        // obviously this will change when we have actual intelligent things to put here
        break;
    }
}

} // namespace flywheel
