#ifndef _COMMANDS_H
#define _COMMANDS_H

#include "types.h"

#define BEMF_IMAGE_START   	0x11003000
#define HALL_IMAGE_START   	0x1100A000
#define FOC_IMAGE_START    	0x11010000
#define BLINK_IMAGE_START  	0x11011000

void changeMode(uint8 requestedMode);

#endif
