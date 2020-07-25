#include "can.h"
#include <string.h>

#include "information/can_protocol.hpp"
#include "information/device.h"

namespace userCAN {

static device_t* gcan_devices;
// Initializes pointer describing an empty struct of devices in the CAN loop
static volatile uint32_t unknown_message = 0;
// Counter for any unreadable messages (Debug purposes)

static CAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8]; // data field for

static bool initialized = false;

static void motor_Decode(motorFeedback_t* fb, const uint8_t* raw) {
    fb->rotor_angle = ((raw[0] << 8) | raw[1]);
    fb->rotor_speed = ((raw[2] << 8) | raw[3]);
    fb->torque_current = ((raw[4] << 8) | raw[5]);
    fb->temp = raw[6];
}
// decodes the "data" field in a CAN message returned by DJI motor/esc's

int8_t deviceInit(device_t* can_devices, osThreadId_t* motor_alert, uint8_t motor_alert_len) {

    if (can_devices == NULL) {
        return DEVICE_ERR_NULL;
    }
    if (motor_alert == NULL) {
        return DEVICE_ERR_NULL;
    }
    if (initialized) {
        return DEVICE_ERR_INITED;
        /* "initialized" is set to false by default, 
        if it's already true then something went wrong
        and initialization already happened */
    }

    can_devices->motor_alert_len = motor_alert_len;
    can_devices->motor_alert = motor_alert;

    CAN_FilterTypeDef can_filter = {0};

    can_filter.FilterBank = 0;
    can_filter.FilterIdHigh = 0;
    can_filter.FilterIdLow = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterMaskIdHigh = 0;
    can_filter.FilterMaskIdLow = 0;
    can_filter.FilterActivation = ENABLE;
    can_filter.SlaveStartFilterBank = 14;
    can_filter.FilterFIFOAssignment = CAN_MOTOR_CAN_RX_FIFO;
    // Creates filter with nothing in it

    HAL_CAN_ConfigFilter(&hcan1, &can_filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

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
    int16_t motor1 = m1 * CAN_M3508_MAX_ABS_VOLT; // honestly dont know what this scalar describes
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

int8_t motor_ControlGimbal(float32_t yaw, float32_t pitch) {
    int16_t yaw_motor = yaw * CAN_GM6020_MAX_ABS_VOLT;
    int16_t pitch_motor = pitch * CAN_GM6020_MAX_ABS_VOLT;

    CAN_TxHeaderTypeDef tx_header;

    tx_header.StdId = CAN_GM6020_RECEIVE_ID_EXTAND;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    uint8_t tx_data[8];
    tx_data[0] = yaw_motor >> 8;
    tx_data[1] = yaw_motor;
    tx_data[2] = pitch_motor >> 8;
    tx_data[3] = pitch_motor;
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0);

    return DEVICE_OK;
}

int8_t motor_ControlFeeder(float32_t feeder) {
    int16_t feeder_motor = feeder * CAN_M2006_MAX_ABS_VOLT;

    CAN_TxHeaderTypeDef tx_header;

    tx_header.StdId = CAN_M3508_M2006_RECEIVE_ID_EXTAND;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    uint8_t tx_data[8];
    tx_data[0] = feeder_motor >> 8;
    tx_data[1] = feeder_motor;
    tx_data[2] = 0;
    tx_data[3] = 0;
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0);

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

void RxFifo0MsgPendingCallback(void) {
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);

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
        // index = rx_header.StdId - CAN_M3508_M1_ID;
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

    // if (motor_received > CAN_CHASSIS_NUM_MOTOR) {
    //     for (uint8_t i = 0; i < gcan_devices->motor_alert_len; i++) {
    //         if (gcan_devices->motor_alert[i]) {
    //             osThreadFlagsSet(gcan_devices->motor_alert, SIGNAL_CAN_MOTOR_RECV);
    //         }
    //     }
    //     motor_received = 0;
    // }
}

} // namespace userCAN
