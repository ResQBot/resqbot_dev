#ifndef COMMON_MC_H
#define COMMON_MC_H

#include "types.h"

// Number of messages for dataset transfer
#define NUMBEROF_MESSAGES_FOC					(46u)
#define NUMBEROF_MESSAGES_BEMF				(30u)
#define NUMBEROF_MESSAGES_HALL				(22u)
#define NUMBEROF_BYTES_FOC						(NUMBEROF_MESSAGES_FOC * 2u)
#define NUMBEROF_BYTES_BEMF						(NUMBEROF_MESSAGES_BEMF * 2u)
#define NUMBEROF_BYTES_HALL						(NUMBEROF_MESSAGES_HALL * 2u)

// Motor errors
#define EMO_ERROR_NONE								(0u)
#define EMO_ERROR_MOTOR_INIT					(1u)
#define EMO_ERROR_MOTOR_NOT_STOPPED		(2u)
#define EMO_ERROR_MOTOR_NOT_STARTED		(3u)

// Motor states
#define EMO_MOTOR_STATE_UNINIT 				(0u)
#define EMO_MOTOR_STATE_STOP   				(1u)
#define EMO_MOTOR_STATE_START  				(2u)
#define EMO_MOTOR_STATE_RUN    				(3u)
#define EMO_MOTOR_STATE_FAULT  				(4u)
#define EMO_MOTOR_STATE_SWITCH 				(5u)
#define EMO_MOTOR_STATE_SPEEDTEST1 		(6u)
#define EMO_MOTOR_STATE_SPEEDTEST2 		(7u)
#define EMO_MOTOR_STATE_START0  			(8u)
#define EMO_MOTOR_STATE_START1  			(9u)
#define EMO_MOTOR_STATE_START2  			(10u)
#define EMO_MOTOR_STATE_START3  			(11u)
#define EMO_MOTOR_STATE_START4  			(12u)
#define EMO_MOTOR_STATE_START5  			(13u)
#define EMO_MOTOR_STATE_START6  			(14u)

void changeMode(uint8 reqMode);
void checkErrorsMC(uint16* motorStartError);
uint8 CRC8(uint8 *data, uint8 len);
uint8 isValueInArray(uint8 val, const uint8 *indices16bit, const uint8 size);

#endif
