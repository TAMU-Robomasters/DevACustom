#include "can.h"
#include "cmsis_os.h"
#include "main.h"
#include "stm32f4xx_hal_can.h"
#include <string.h>

#include "information/can_protocol.hpp"
#include "information/device.hpp"

#include "subsystems/chassis.hpp"
#include "subsystems/feeder.hpp"
#include "subsystems/gimbal.hpp"

float currYAWTime;
float lastYAWTime;
float lastYAWDataTime;
float currCANTime;
float lastCANTime;
float lastCANLoopTime;
float canFillLevel;

namespace userCAN {

static device_t* can_devices_ptr;
// Initializes pointer describing an empty struct of devices in the CAN loop
static volatile uint32_t unknown_message = 0;
// Counter for any unreadable messages (Debug purposes)

static CAN_RxHeaderTypeDef rx_header;
//rx_header.stdId = 4;
static uint8_t rx_data[8]; // data field for incoming messages (from DJI motors/escs)

static bool initialized = false;

static void motor_Decode(motorFeedback_t* fb, const uint8_t* raw) {
    fb->rotor_angle = ((raw[0] << 8) | raw[1]);
    fb->rotor_speed = ((raw[2] << 8) | raw[3]);
    fb->torque_current = ((raw[4] << 8) | raw[5]);
    fb->temp = raw[6];
}
// decodes the "data" field in a CAN message returned by DJI motor/esc's

void canFilterInit() {
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_FilterFIFO0;
    can_filter_st.SlaveStartFilterBank = 14; //can1(0-13)和can2(14-27)分别得到一半的filter
    can_filter_st.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) == 0x00U) {
        // indicator here
    } // Creates filter that doesn't filter anything

    if (HAL_CAN_Start(&hcan1) == 0x00U) {
        // indicator here
    }

    can_filter_st.SlaveStartFilterBank = 15;
    can_filter_st.FilterBank = 17;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
}

int8_t deviceInit(device_t* can_devices) {

    if (can_devices == NULL) {
        return DEVICE_ERR_NULL;
    }
    // if (motor_alert == NULL) {
    //     return DEVICE_ERR_NULL;
    // }
    if (initialized) {
        return DEVICE_ERR_INITED;
        /* "initialized" is set to false by default, 
        if it's already true then something went wrong
        and initialization already happened */
    }

    canFilterInit();

    can_devices_ptr = can_devices;
    initialized = true;
    return DEVICE_OK;
}

device_t* getDevices(void) {
    if (initialized) {
        return can_devices_ptr;
    }
    return NULL;
}

int8_t motor_ControlChassis(float32_t m1, float32_t m2, float32_t m3, float32_t m4, CAN_HandleTypeDef can) {
    int16_t motor1 = static_cast<int16_t>((m1 * M3508_MAX_CURRENT) / 100); // scalar describes current cap but is named volt for some reason
    int16_t motor2 = static_cast<int16_t>((m2 * M2006_MAX_CURRENT) / 100);
    int16_t motor3 = static_cast<int16_t>((m3 * M2006_MAX_CURRENT) / 100);
    int16_t motor4 = static_cast<int16_t>((m4 * M2006_MAX_CURRENT) / 100);

    CAN_TxHeaderTypeDef tx_header;

    tx_header.StdId = CAN_M3508_M2006_RECEIVE_ID_BASE;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    uint8_t tx_data[8];
    tx_data[0] = motor1 >> 8;
    tx_data[1] = motor1;
    tx_data[2] = motor2 >> 8;
    tx_data[3] = motor2;
    tx_data[4] = motor3 >> 8;
    tx_data[5] = motor3;
    tx_data[6] = motor4 >> 8;
    tx_data[7] = motor4;

    HAL_CAN_AddTxMessage(&can, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0);

    return DEVICE_OK;
}

int8_t motor_ControlGimbFeed(float32_t yaw, float32_t pitch, float32_t feeder, CAN_HandleTypeDef can) {
    int16_t yawMotor = static_cast<int16_t>((yaw * CAN_GM6020_MAX_ABS_VOLT) / 100);
    int16_t pitchMotor = static_cast<int16_t>((pitch * CAN_GM6020_MAX_ABS_VOLT) / 100);
    int16_t feederMotor = static_cast<int16_t>((feeder * M2006_MAX_CURRENT) / 100);

    CAN_TxHeaderTypeDef tx_header;

    tx_header.StdId = CAN_GM6020_RECEIVE_ID_BASE;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    uint8_t tx_data[8];
    tx_data[0] = yawMotor >> 8;
    tx_data[1] = yawMotor;
    tx_data[2] = pitchMotor >> 8;
    tx_data[3] = pitchMotor;
    tx_data[4] = feederMotor >> 8;
    tx_data[5] = feederMotor;
    tx_data[6] = 0;
    tx_data[7] = 0;

    HAL_CAN_AddTxMessage(&can, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0);

    return DEVICE_OK;
}

int8_t motor_QuickIdSetMode(void) {
    CAN_TxHeaderTypeDef tx_header;

    tx_header.StdId = CAN_M3508_M2006_ID_SETTING_ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;

    uint8_t tx_data[8];

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0);

    return DEVICE_OK;
}

bool getMessage(CAN_HandleTypeDef* can) {
    if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &rx_header, rx_data) != 0x00U) {
        //HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
        return false;
    }

    // uint8_t index;
    switch (rx_header.StdId) {
    case M3508_M1_ID:
        // motor_Decode(&(can_devices_ptr->feeder_fb), rx_data);
        motor_Decode(chassis::c1Motor.getFeedback(), rx_data);
        break;

    case M2006_INDEXER_ID:
        motor_Decode(feeder::indexer.getFeedback(), rx_data);
        break;

    case M2006_AGITATOR_RIGHT_ID:
        motor_Decode(feeder::agitatorRight.getFeedback(), rx_data);
        break;

    case M2006_AGITATOR_LEFT_ID:
        motor_Decode(feeder::agitatorLeft.getFeedback(), rx_data);
        break;

    case GM6020_YAW_ID:
        motor_Decode(gimbal::yawMotor.getFeedback(), rx_data);
        lastYAWTime = currYAWTime;
        currYAWTime = HAL_GetTick();

        lastYAWDataTime = currYAWTime - lastYAWTime;
        break;

    case GM6020_PIT_ID:
        motor_Decode(gimbal::pitchMotor.getFeedback(), rx_data);
        break;

    default:
        unknown_message++;
        // HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
        break;
    }
    return true;
}

void task() {

    userInit::canInit();

    for (;;) {
        receive();

        send();
        // for sending messages over CAN to motors
        lastCANTime = currCANTime;
        currCANTime = HAL_GetTick();

        lastCANLoopTime = currCANTime - lastCANTime;

        osDelay(1);
    }
}

void receive() {
    canFillLevel = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
    //if (canFillLevel > 0) {
    //HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
    //while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0){
    for (unsigned int i = 0; i < canFillLevel; i++) {
        userCAN::getMessage(&hcan1);
    }
    //}
}

void send() {
    if (operatingType == primary) {
        userCAN::motor_ControlGimbFeed(gimbal::yawMotor.getPower(), gimbal::pitchMotor.getPower(), feeder::indexer.getPower(), hcan1);
    }

    if (operatingType == secondary) {
        userCAN::motor_ControlChassis(chassis::c1Motor.getPower(), feeder::agitatorLeft.getPower(), feeder::agitatorRight.getPower(), 0, hcan1);
        userCAN::motor_ControlGimbFeed(gimbal::yawMotor.getPower(), gimbal::pitchMotor.getPower(), feeder::indexer.getPower(), hcan1);
    }
}

} // namespace userCAN
