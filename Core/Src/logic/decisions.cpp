#include "logic/decisions.hpp"
#include "cmsis_os.h"

float position = 0;
float testSpeed = METERSPS_TO_RPM(0.25f);

namespace decisions {

void task() {
    osDelay(2000);

    for (;;) {
        position++;
        chassis::currState = chassis::chassisStates::yield;
        // chassis::velocityMove(5, 1000); // to 6in on the rail
				osDelay(1000);
        // chassis::velocityMove(-5, 1000); // to 6in on the rail
				osDelay(1000);
        // chassis::velocityMove(5, 1000); // to 6in on the rail
				osDelay(1000);
        // chassis::currState = chassis::chassisStates::yield;
        // chassis::profiledMove(IN_TO_METER(9)); // 5in
        // chassis::currState = chassis::chassisStates::notRunning;
        position++;
			  // chassis::velocityMove(-5, 1000); // to 6in on the rail
				osDelay(1000);
        // osDelay(10000);
    }
}

}; // namespace decisions
