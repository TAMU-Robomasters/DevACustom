#include "subsystems/feeder.hpp"
#include "can.h"
#include "information/can_protocol.hpp"
#include "information/rc_protocol.h"
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
    currState = notRunning;
    if (getSwitch(switchType::right) == switchPosition::up) {
        currState = running;
    }

		velPidF1.setTarget(-40);
		f1Output = f1Motor.getSpeed();
}

void act() {
    switch (currState) {
    case notRunning:
        f1Motor.setPower(0);
        break;

    case running:
        f1Motor.setPower(velPidF1.loop(f1Motor.getSpeed()));
        // f1Motor.setPower(20);
        // obviously this will change when we have actual intelligent things to put here
        break;
    }
}
// set power to global variable here, message is actually sent with the others in the CAN task

} // namespace feeder
