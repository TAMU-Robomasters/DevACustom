#include "subsystems/feeder.hpp"
#include "can.h"
#include "information/can_protocol.hpp"
#include "information/uart_protocol.hpp"
#include "init.hpp"
#include "main.h"

namespace feeder {

feederStates currState = notRunning;

float feederPower = 0;

uint16_t angle = 0;
int16_t speed = 0;
int16_t torque_current = 0;
int8_t temp = 0;

void task() {

    //osDelay(5000);

    for (;;) {
        update();

        act();

        osDelay(10);
    }
}

void update() {

    angle = canDevice.feeder_fb.rotor_angle;
    speed = canDevice.feeder_fb.rotor_speed / 36;
    torque_current = canDevice.feeder_fb.torque_current;

    if (true) {
        currState = running;
    }
}

void act() {
    switch (currState) {
    case notRunning:
        feederPower = 0;
        break;

    case running:
        feederPower = 0.9;
        // obviously this will change when we have actual intelligent things to put here
        break;
    }
}
// set power to global variable here, message is actually sent with the others in the CAN task

void indicator() {

    HAL_GPIO_WritePin(GPIOG, LED_A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, LED_B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, LED_C_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, LED_D_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, LED_E_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, LED_F_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, LED_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, LED_H_Pin, GPIO_PIN_SET);

    int increment = 550 / 8;
    if (speed > increment * 1) {
        HAL_GPIO_WritePin(GPIOG, LED_A_Pin, GPIO_PIN_RESET);
    }
    if (speed > increment * 2) {
        HAL_GPIO_WritePin(GPIOG, LED_B_Pin, GPIO_PIN_RESET);
    }
    if (speed > increment * 3) {
        HAL_GPIO_WritePin(GPIOG, LED_C_Pin, GPIO_PIN_RESET);
    }
    if (speed > increment * 4) {
        HAL_GPIO_WritePin(GPIOG, LED_D_Pin, GPIO_PIN_RESET);
    }
    if (speed > increment * 5) {
        HAL_GPIO_WritePin(GPIOG, LED_E_Pin, GPIO_PIN_RESET);
    }
    if (speed > increment * 6) {
        HAL_GPIO_WritePin(GPIOG, LED_F_Pin, GPIO_PIN_RESET);
    }
    if (speed > increment * 7) {
        HAL_GPIO_WritePin(GPIOG, LED_G_Pin, GPIO_PIN_RESET);
    }
    if (speed > increment * 8) {
        HAL_GPIO_WritePin(GPIOG, LED_H_Pin, GPIO_PIN_RESET);
    }
}

} // namespace feeder
