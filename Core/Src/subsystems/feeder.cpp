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
float indexerP;
float indexerI;
float indexerD;
float indexerPidOut, agitatorLeftPidOut, agitatorRightPidOut;
float unJamTimer;
float runningTimer;

namespace feeder {

feederStates currState = notRunning;
	
filter::Kalman feederVelFilter(0.05, 16.0, 1023.0, 0.0);

pidInstance velPidAgitatorLeft(pidType::velocity, 0.7, 0.00, 0.01);
pidInstance velPidAgitatorRight(pidType::velocity, 0.7, 0.00, 0.01);
pidInstance velPidIndexer(pidType::velocity, 1.5, 0.000, 0.01);

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
    //currState = notRunning;
    if (switchIsRising(switchType::left, switchPosition::up)) {
        currState = running;
    }
    if (switchIsFalling(switchType::left, switchPosition::up)){
        currState = notRunning;
    }

    //f1Output = agitatorRight.getSpeed();
    indexerI = velPidIndexer.getIntegral();
    indexerPidOut = velPidIndexer.getOutput();
    //agitatorLeftPidOut = velPidAgitatorLeft.getOutput();
    //agitatorRightPidOut = velPidAgitatorRight.getOutput();
}

void act() {
    switch (currState) {
    case notRunning: {
			  HAL_GPIO_WritePin(GPIOH, POWER4_CTRL_Pin, GPIO_PIN_RESET);	
        agitatorLeft.setPower(0);
        agitatorRight.setPower(0);
        indexer.setPower(0);
        break;
    }
    case running: {
			  HAL_GPIO_WritePin(GPIOH, POWER4_CTRL_Pin, GPIO_PIN_SET);
        //if (runningTimer <= 2000){
        float feederSpeed = -25; //Was 150 before speeding 
        // velPidAgitatorLeft.setTarget(feederSpeed * (2.0f / 7.0f));
        // velPidAgitatorRight.setTarget(-feederSpeed * (2.0f / 7.0f));
        velPidIndexer.setTarget(feederSpeed);

        // agitatorRight.setPower(velPidAgitatorRight.loop(agitatorRight.getSpeed()));
        // agitatorLeft.setPower(velPidAgitatorLeft.loop(agitatorLeft.getSpeed()));
        indexer.setPower(velPidIndexer.loop(indexer.getSpeed()));

        runningTimer += 10;
        //}
        // else {
        //     runningTimer = 0;
        //     currState = unJam;
        // }
        break;
    }
    case unJam: {
        if(unJamTimer <= 350){
            // float feederSpeed = -100;
            //velPidAgitatorLeft.setTarget(feederSpeed * 1.2f);
            //velPidAgitatorRight.setTarget(-feederSpeed * 1.2f);
            // velPidIndexer.setTarget(feederSpeed);

            //agitatorRight.setPower(velPidAgitatorRight.loop(agitatorRight.getSpeed()));
            //agitatorLeft.setPower(velPidAgitatorLeft.loop(agitatorLeft.getSpeed()));
            // indexer.setPower(velPidIndexer.loop(indexer.getSpeed()));

            unJamTimer += 10;
					
        }
        else {
            unJamTimer = 0;
            currState = running;
        }
        break;
    }
    }
}
// set power to global variable here, message is actually sent with the others in the CAN task

} // namespace feeder
