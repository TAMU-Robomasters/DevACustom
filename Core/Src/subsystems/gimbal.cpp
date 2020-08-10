#include "subsystems/gimbal.hpp"
#include "init.hpp"

namespace gimbal {

gimbalStates currState = notRunning;

float yawPower = 0;
float pitchPower = 0;

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
        yawPower = pitchPower = 0;
        break;

    case running:
        yawPower = pitchPower = 0;
        // obviously this will change when we have actual intelligent things to put here
        break;
    }
}

} // namespace gimbal
