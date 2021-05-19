#pragma once

#include "sdio.h"

#define ERR_MOUNT_MKFS (1)
#define ERR_OPEN (2)

extern uint8_t errorCode;

extern void checkOperatingType();

extern void sdTestFunc();
