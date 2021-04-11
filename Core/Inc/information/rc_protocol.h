#pragma once
//#include "bsp_rc.h"
//#include "struct_typedef.h"
#include "usart.h"
#include "main.h"
/* 
	Code heavily taken from DJI
*/
#define UART_RX_DMA_SIZE (1024)
#define DBUS_MAX_LEN (50)
#define DBUS_BUFLEN (18)
#define DBUS_HUART huart1 /* for dji remote controler reciever */

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

#define GET_BIT(a, b) (a >> b & 1)
// shifts input (b) to right by (a) digits and does bitwise AND with 1

typedef __packed struct {
    __packed struct {
        int16_t ch[5]; // analog joysticks
        uint8_t s[2];  // three stage switches
    } rc;
    __packed struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    __packed struct {
        uint16_t v;
    } key;

} RC_ctrl_t;

typedef enum {
    rightX = 0,
    rightY = 1,
    leftX = 2,
    leftY = 3,
} joystickAxis;

typedef enum {
    right = 0,
    left = 1,
} switchType;

typedef enum {
    up = 1,
    mid = 3,
    down = 2,
} switchPosition;

typedef enum {
    btnW = 0,
    btnS = 1,
    btnA = 2,
    btnD = 3,
    btnShift = 4,
    btnCtrl = 5,
    btnQ = 6,
    btnE = 7,
    btnR = 8,
    btnF = 9,
    btnG = 10,
    btnZ = 11,
    btnX = 12,
    btnC = 13,
    btnV = 14,
    btnB = 15,
    btnMouseL,
    btnMouseR,
} btnType;

extern RC_ctrl_t rcDataStruct;
extern RC_ctrl_t lastRcDataStruct;

extern bool btnIsRising(btnType btn);
extern bool btnIsFalling(btnType btn);

extern uint8_t getSwitch(switchType sw);
extern float getJoystick(joystickAxis joy);

extern RC_ctrl_t* getRCData();

extern void processDMAData();

void uart_receive_handler(UART_HandleTypeDef* huart);

extern void RCInit();
