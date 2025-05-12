
/*
 * Global variables needed for changing BEMF parameters with arduino
 * initialized with standard values from defines
 */
#ifndef _GLOBVARS_BEMF_H
#define _GLOBVARS_BEMF_H

#include "types.h"
#include "commands.h"
#include "Emo_BEMF.h"

extern const uint8 indices_16bit[];
extern const uint8 indices_16bit_size;

/* for changing BEMF_PWM_FREQ  */
extern float ccu6_t12_freq;
extern uint16 ccu6_t12_period_sel;
extern float pwm_period;

extern uint16 ccu6_tctr0_bit7;
extern uint16 ccu6_t12period;
extern uint16 ccu6_t12pr;

extern float ccu6_cc60sr;
extern float ccu6_cc61sr;
extern float ccu6_cc62sr;

/* for changing BEMF_START_SPEED_PWM_MIN_OFFSET and BEMF_END_SPEED_PWM_MIN */
extern float bemfEndSpeedPwmMin;

typedef union
{
	uint16 dataarray_BEMF[NROFMESSAGES];
	Tdatastruct_BEMF datastruct_BEMF;
} data_BEMF_union;
extern data_BEMF_union rxtxbuffer;

#endif
