#include "subsystems/feeder.hpp"
#include "can.h"
#include "cmsis_os.h"
#include "information/can_protocol.hpp"
#include "information/rc_protocol.h"
#include "information/uart_protocol.hpp"
#include "init.hpp"
#include "main.h"
#include "stm32f4xx_hal.h"

float f1Output;
float indexerPidOut, agitatorLeftPidOut, agitatorRightPidOut;

namespace feeder {

feederStates currState = notRunning;
	
filter::Kalman feederVelFilter(0.05, 16.0, 1023.0, 0.0);

pidInstance velPidAgitatorLeft(pidType::velocity, 0.2, 0.00, 0.01);
pidInstance velPidAgitatorRight(pidType::velocity, 0.2, 0.00, 0.01);
pidInstance velPidIndexer(pidType::velocity, 0.5, 0.00, 0.01);

feederMotor agitatorLeft(userCAN::M2006_AGITATOR_LEFT_ID, velPidAgitatorLeft, feederVelFilter);
feederMotor agitatorRight(userCAN::M2006_AGITATOR_RIGHT_ID, velPidAgitatorRight, feederVelFilter);
feederMotor indexer(userCAN::M2006_INDEXER_ID, velPidIndexer, feederVelFilter);

void task() {
    HAL_GPIO_WritePin(POWER2_CTRL_GPIO_Port, POWER2_CTRL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(POWER4_CTRL_GPIO_Port, POWER4_CTRL_Pin, GPIO_PIN_SET);

    for (;;) {
        update();

        act();

        osDelay(10);
    }
}

void update() {
    currState = notRunning;
    if (getSwitch(switchType::left) == switchPosition::up) {
        currState = running;

        float feederSpeed = 150;

        velPidAgitatorLeft.setTarget(feederSpeed * (4.75f / 7.0f));
        velPidAgitatorRight.setTarget(-feederSpeed * (4.75f / 7.0f));
        velPidIndexer.setTarget(feederSpeed);
    }

    f1Output = agitatorRight.getSpeed();
		indexerPidOut = velPidIndexer.getOutput();
		agitatorLeftPidOut = velPidAgitatorLeft.getOutput();
		agitatorRightPidOut = velPidAgitatorRight.getOutput();

}

void act() {
    switch (currState) {
    case notRunning:
        agitatorLeft.setPower(0);
        agitatorRight.setPower(0);
        indexer.setPower(0);
        break;

    case running:
        agitatorRight.setPower(velPidAgitatorRight.loop(agitatorRight.getSpeed()));
        agitatorLeft.setPower(velPidAgitatorLeft.loop(agitatorLeft.getSpeed()));
        indexer.setPower(velPidIndexer.loop(indexer.getSpeed()));
        // obviously this will change when we have actual intelligent things to put here
        break;
    }
}
// set power to global variable here, message is actually sent with the others in the CAN task

} // namespace feeder
