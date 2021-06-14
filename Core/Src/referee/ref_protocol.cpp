#include "referee/ref_protocol.hpp"

SerialRxState djiSerialRxState;
SerialMessage newMessage;
SerialMessage mostRecentMessage;

uint16_t frameCurrReadByte;
uint8_t frameHeader[FRAME_HEADER_LENGTH];
bool rxCrcEnabled;
uint16_t currCrc16 = 0;
