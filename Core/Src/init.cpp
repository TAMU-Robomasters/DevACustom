#include "init.hpp"
#include "main.h"

userCAN::device_t canDevice;

namespace userInit {

void initialize() {
    if (userCAN::deviceInit(&canDevice) == 0) {
        // HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
    } // inializes the set of devices on our CAN loop
}

} // namespace userInit
