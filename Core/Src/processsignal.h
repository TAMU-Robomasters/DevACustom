#include "signal.h"

//this signal process the signal from the controller
//and store them in a struct signal.

//PreSignal is the address of pre-processed signal, 18 bytes
//Signal is the processed signal, in a struct
//developed based on sources from DJI
void processsignal(uint8_t *PreSignal, struct Signal *signal)
{
    signal->ch0 =  ((int16_t)PreSignal[0] | ((int16_t)PreSignal[1] << 8)) & 0x07FF;
    signal->ch1 = (((int16_t)PreSignal[1] >> 3) | ((int16_t)PreSignal[2] << 5)) & 0x07FF;
    signal->ch2 = (((int16_t)PreSignal[2] >> 6) | ((int16_t)PreSignal[3] << 2) | ((int16_t)PreSignal[4] << 10)) & 0x07FF;;
    signal->ch3 = (((int16_t)PreSignal[4] >> 1) | ((int16_t)PreSignal[5]<<7)) & 0x07FF;
    signal->s1 = ((PreSignal[5] >> 4) & 0x000C) >> 2;
    signal->s2 = ((PreSignal[5] >> 4) & 0x0003);
    signal->x = ((int16_t)PreSignal[6]) | ((int16_t)PreSignal[7] << 8);
    signal->y = ((int16_t)PreSignal[8]) | ((int16_t)PreSignal[9] << 8);
    signal->z = ((int16_t)PreSignal[10]) | ((int16_t)PreSignal[11] << 8);
    signal->press_l = PreSignal[12];
    signal->press_r = PreSignal[13];
    signal->v = ((int16_t)PreSignal[14]);// | ((int16_t)PreSignal[15] << 8);
}