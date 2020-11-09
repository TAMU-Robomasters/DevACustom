#include <stdint.h>
//File description:
//This struct is used to store the processed control code
//from controller
//when use, please declear a global variable
//see processsignal.h for constructor function

struct Signal
{
    uint16_t ch0; //cho on controller
    uint16_t ch1; //ch1 on controller
    uint16_t ch2; //ch2 on controller
    uint16_t ch3; //ch3 on controller
    uint8_t s1; //s1 on controller
    uint8_t s2; //s2 on controller
    int16_t x; //x direction of mouse
    int16_t y; //y direction of mouse
    int16_t z; //z direction of mouse
    uint8_t press_l; //press l keyboard
    uint8_t press_r; //press r keyboard
    uint16_t v; //wasd on keyboard
    //refer to user manual of controller
};