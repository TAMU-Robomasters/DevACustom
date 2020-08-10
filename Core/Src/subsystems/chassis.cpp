#include "subsystems/chassis.hpp"
#include "init.hpp"

namespace chassis {

chassisStates currState = notRunning;

float c1Power = 0;
float c2Power = 0;
float c3Power = 0;
float c4Power = 0;

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
        c1Power = c2Power = c3Power = c4Power = 0;
        break;

    case followGimbal:
        c1Power = c2Power = c3Power = c4Power = 0;
        // obviously this will change when we have things to put here
        break;

    case manual:
        c1Power = c2Power = c3Power = c4Power = 0;
        // obviously this will change when we have things to put here
        break;
    }
}

} // namespace chassis
