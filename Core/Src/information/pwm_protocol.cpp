#include "information/pwm_protocol.hpp"
#include "main.h"
#include "tim.h"

namespace userPWM {

void setPower(TIM_HandleTypeDef* tim, int tim_channel, float power) {
    // effective max and min are 1083, 533
    int upperRange = 1083;
    int lowerRange = 532;
    int dutyRange = upperRange - lowerRange;

    int duty = int(532 + power * dutyRange);

    switch (tim_channel) {
    case 1:
        tim->Instance->CCR1 = duty;
        break;
    case 2:
        tim->Instance->CCR2 = duty;
        break;
    case 3:
        tim->Instance->CCR3 = duty;
        break;
    case 4:
        tim->Instance->CCR4 = duty;
        break;
    }
}

void initESC(TIM_HandleTypeDef* tim, int tim_channel, GPIO_TypeDef* GPIOx, int GPIO_Pin) {
    // htim2.Instance->CCR1 = 1000;
    userPWM::setPower(tim, tim_channel, 1);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    osDelay(1000);
    userPWM::setPower(tim, 1, -.058);
    // htim2.Instance->CCR1 = 500;
    osDelay(1000);
}

void initChannels() {
    /* Open 4 sets of PWM waves respectively */
    /**TIM2 GPIO Configuration    
	PA1     ------> TIM2_CH2
	PA0     ------> TIM2_CH1
	PA2     ------> TIM2_CH3
	PA3     ------> TIM2_CH4 
	*/
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    /**TIM4 GPIO Configuration    
	PD15     ------> TIM4_CH4
	PD14     ------> TIM4_CH3
	PD13     ------> TIM4_CH2
	PD12     ------> TIM4_CH1 
	*/
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    /**TIM5 GPIO Configuration    
	PI0     ------> TIM5_CH4
	PH12     ------> TIM5_CH3
	PH11     ------> TIM5_CH2
	PH10     ------> TIM5_CH1 
	*/
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

    /**TIM8 GPIO Configuration    
	PI7     ------> TIM8_CH3
	PI6     ------> TIM8_CH2
	PI5     ------> TIM8_CH1
	PI2     ------> TIM8_CH4 
	*/
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

} // namespace userPWM
