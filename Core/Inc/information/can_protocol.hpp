#pragma once
#include "can.h"
#include "cmsis_os.h"
#include "main.h"
#include "stm32f4xx_hal_can.h"

//#include "cmsis_os2.h"

/* 
	Our custom functions for CAN; heavily inspired by QDU
*/

typedef float float32_t;
typedef void* osThreadId_t;

/* Fixed-size types, underlying types depend on word size and compiler.  */
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int int16_t;
typedef unsigned short int uint16_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;

// Motor Constants
#define CAN_GM6020_FEEDBACK_ID_BASE 0x205
#define CAN_GM6020_RECEIVE_ID_BASE 0x1ff
#define CAN_GM6020_RECEIVE_ID_EXTEND 0x2ff

#define CAN_M3508_M2006_FEEDBACK_ID_BASE 0x201
#define CAN_M3508_M2006_RECEIVE_ID_BASE 0x200
#define CAN_M3508_M2006_RECEIVE_ID_EXTEND 0x1ff
#define CAN_M3508_M2006_ID_SETTING_ID 0x700

#define CAN_MOTOR_MAX_NUM 9
#define CAN_CHASSIS_NUM_MOTOR 4
#define CAN_GIMBAL_NUM_MOTOR 5

#define CAN_MOTOR_MAX_ENCODER 8191

#define CAN_MOTOR_CAN_RX_FIFO CAN_RX_FIFO0

namespace userCAN {

typedef enum motorId_t {
    M3508_M1_ID = 0x201, /* 1 */
    M2006_AGITATOR_LEFT_ID = 0x202,         /* 2 */
    M2006_AGITATOR_RIGHT_ID = 0x203,  /* 3 */
    /* https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.01.pdf */

    GM6020_YAW_ID = 0x205, /* 1 */
    GM6020_PIT_ID = 0x206, /* 2 */
    /* https://rm-static.djicdn.com/tem/3724/RoboMaster%20GM6020%20Brushless%20DC%20Motor%20User%20Guide.pdf */

    M2006_INDEXER_ID = 0x207,   /* 7 */
    /* https://rm-static.djicdn.com/tem/17348/RM%20M2006%20P36%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E5%87%8F%E9%80%9F%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.pdf */
} motorId_t;
// Initializing motor CAN ID's with their numbers, accessible through type "motorID_t"

typedef struct motorFeedback_t {
    uint16_t rotor_angle;
    int16_t rotor_speed;
    int16_t torque_current;
    uint8_t temp;
} motorFeedback_t;
// Types of sensor feedback returnable from DJI Motors; accessible through type "motorFeedback_t"

typedef struct device_t {
    osThreadId_t* motor_alert;
    uint8_t motor_alert_len;

    motorFeedback_t chassis_motor_fb[4];

    motorFeedback_t feeder_fb;

    struct {
        motorFeedback_t yaw_fb;
        motorFeedback_t pit_fb;
    } gimbal_motor_fb;

} device_t;
// Types of motors with CAN data feedback; accessible within the type "device_t"

extern int8_t deviceInit(
    device_t* can_devices);
// Initialization of the CAN devices, the CAN filter, and CAN notifications(?)

extern void task();
// called in freertos.cpp

extern void receive();
extern void send();
// !!!

extern device_t* getDevices(void);
// Getter for "can_devices_ptr" of "type device_t"

extern int8_t motor_ControlChassis(float32_t m1, float32_t m2, float32_t m3, float32_t m4, CAN_HandleTypeDef can);
// Sends message via CAN with current values for each motor
extern int8_t motor_ControlGimbFeed(float32_t yaw, float32_t pitch, float32_t feeder, CAN_HandleTypeDef can);
// Sends message via CAN with voltage/current values for yaw and pitch motors

extern int8_t motor_QuickIdSetMode(void);
// Starts the ID setting proces of M3508/M2006 motors? dunno tbh

extern bool getMessage(CAN_HandleTypeDef* hcan);

} // namespace userCAN
