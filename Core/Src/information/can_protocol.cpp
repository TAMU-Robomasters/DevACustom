#include "can.h"
#include "cmsis_os.h"
#include "main.h"
#include "stm32f4xx_hal_can.h"
#include <string.h>

#include "information/can_protocol.hpp"
#include "information/device.h"

#include "subsystems/chassis.hpp"
#include "subsystems/feeder.hpp"
#include "subsystems/gimbal.hpp"

namespace userCAN {

static device_t* gcan_devices;
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

    gcan_devices = can_devices;
    initialized = true;
    return DEVICE_OK;
}

device_t* getDevices(void) {
    if (initialized) {
        return gcan_devices;
    }
    return NULL;
}

int8_t motor_ControlChassis(float32_t m1, float32_t m2, float32_t m3, float32_t m4) {
    int16_t motor1 = m1 * CAN_M3508_MAX_ABS_VOLT; // scalar describes current cap but is named volt for some reason
    int16_t motor2 = m2 * CAN_M3508_MAX_ABS_VOLT;
    int16_t motor3 = m3 * CAN_M3508_MAX_ABS_VOLT;
    int16_t motor4 = m4 * CAN_M3508_MAX_ABS_VOLT;

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

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0);

    return DEVICE_OK;
}

int8_t motor_ControlGimbFeed(float32_t yaw, float32_t pitch, float32_t feeder) {
    int16_t yaw_motor = yaw * CAN_GM6020_MAX_ABS_VOLT;
    int16_t pitch_motor = pitch * CAN_GM6020_MAX_ABS_VOLT;
    int16_t feeder_motor = feeder * CAN_M2006_MAX_ABS_VOLT;

    CAN_TxHeaderTypeDef tx_header;

    tx_header.StdId = CAN_GM6020_RECEIVE_ID_BASE;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    uint8_t tx_data[8];
    tx_data[0] = yaw_motor >> 8;
    tx_data[1] = yaw_motor;
    tx_data[2] = pitch_motor >> 8;
    tx_data[3] = pitch_motor;
    tx_data[4] = feeder_motor >> 8;
    tx_data[5] = feeder_motor;
    tx_data[6] = 0;
    tx_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0);

    return DEVICE_OK;
}

// int8_t motor_ControlFeeder(float32_t feeder) {
//     int16_t feeder_motor = feeder * CAN_M2006_MAX_ABS_VOLT;
//     // (input range to this function should be from -1 -> 1, it's scaled up)

//     CAN_TxHeaderTypeDef tx_header;

//     tx_header.StdId = CAN_M3508_M2006_RECEIVE_ID_EXTAND;
//     // IF ID 1-4, MAKE THIS BASE (0x200) IF ID 5-8, MAKE THIS EXTAND (0x1FF)
//     tx_header.IDE = CAN_ID_STD;
//     tx_header.RTR = CAN_RTR_DATA;
//     tx_header.DLC = 8;

//     uint8_t tx_data[8];
//     tx_data[0] = feeder_motor >> 8;
//     tx_data[1] = feeder_motor;
//     tx_data[2] = 0;
//     tx_data[3] = 0;
//     tx_data[4] = 0;
//     tx_data[5] = 0;
//     tx_data[6] = 0;
//     tx_data[7] = 0;

//     HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0);

//     return DEVICE_OK;
// }

int8_t motor_QuickIdSetMode(void) {
    CAN_TxHeaderTypeDef tx_header;

    tx_header.StdId = CAN_M3508_M2006_ID_SETTING_ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;

    uint8_t tx_data[8];

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0);

    return DEVICE_OK;
}

void getMessage(/*CAN_HandleTypeDef* hcan*/ void) {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data) == 0x00U) {
        //HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
    }

    // uint8_t index;
    switch (rx_header.StdId) {
    case M3508_M1_ID:
        motor_Decode(&(gcan_devices->chassis_motor_fb[0]), rx_data);
        break;
    case M3508_M2_ID:
        motor_Decode(&(gcan_devices->chassis_motor_fb[1]), rx_data);
        break;
    case M3508_M3_ID:
        motor_Decode(&(gcan_devices->chassis_motor_fb[2]), rx_data);
        break;
    case M3508_M4_ID:
        motor_Decode(&(gcan_devices->chassis_motor_fb[3]), rx_data);
        break;

    case M2006_FEEDER_ID:
        motor_Decode(&(gcan_devices->feeder_fb), rx_data);
        break;

    case GM6020_YAW_ID:
        motor_Decode(&(gcan_devices->gimbal_motor_fb.yaw_fb), rx_data);
        break;

    case GM6020_PIT_ID:
        motor_Decode(&(gcan_devices->gimbal_motor_fb.pit_fb), rx_data);
        break;

    default:
        unknown_message++;
        break;
    }
}

void task() {
    for (;;) {
        receive();

        send();
        // for sending messages over CAN to motors

        osDelay(1);
    }
}

void receive() {
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
        //HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
        userCAN::getMessage();
    }
}

void send() {
    userCAN::motor_ControlChassis(chassis::c1Power,
                                  chassis::c2Power,
                                  chassis::c3Power,
                                  chassis::c4Power);
    userCAN::motor_ControlGimbFeed(gimbal::yawPower,
                                   gimbal::pitchPower,
                                   feeder::feederPower);
}

} // namespace userCAN
