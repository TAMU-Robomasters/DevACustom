#pragma once

#include "cmsis_os.h"
#include "main.h"
#include "stm32f4xx_hal.h"

namespace userPWM {

extern void setPower(TIM_HandleTypeDef* tim, int tim_channel, float power);

extern void initESC(TIM_HandleTypeDef* tim, int tim_channel, GPIO_TypeDef* GPIOx, int GPIO_Pin);

extern void initChannels();

} // namespace userPWM
