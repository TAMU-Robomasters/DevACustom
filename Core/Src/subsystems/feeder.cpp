#include "subsystems/feeder.hpp"
#include "can.h"
#include "information/can_protocol.hpp"
#include "information/uart_protocol.hpp"
#include "init.hpp"
#include "main.h"

float f1Output;

namespace feeder {

feederStates currState = notRunning;
	
filter::Kalman feederVelFilter(0.05, 16.0, 1023.0, 0.0);

pidInstance velPidF1(pidType::velocity, 0.2, 0.001, 0.01);

feederMotor f1Motor(userCAN::M2006_FEEDER_ID, velPidF1, feederVelFilter);

void task() {

    for (;;) {
        update();

        act();

        osDelay(1);
    }
}

void update() {

    if (true) {
        currState = notRunning;
    }

    velPidF1.setTarget(100);
		f1Output = f1Motor.getSpeed();
}

void act() {
    switch (currState) {
    case notRunning:
        // feederPower = 0;
        break;

    case running:
        f1Motor.setPower(velPidF1.loop(f1Motor.getSpeed()));
				//f1Motor.setPower(5);
        // obviously this will change when we have actual intelligent things to put here
        break;
    }
}
// set power to global variable here, message is actually sent with the others in the CAN task

} // namespace feeder
