/*
 * Global variables needed for changing FOC parameters with arduino
 * initialized with standard values from defines
 */
#ifndef _GLOBVARS_FOC_H
#define _GLOBVARS_FOC_H

#include "types.h"
#include "Emo_RAM.h"
#include "..\..\common.h"
#include "..\..\commonMC.h"

extern const uint8 indices_16bit[];
extern const uint8 indices_16bit_size;

/* for changing FOC_PWM_FREQ  */
extern float ccu6_t12_freq;
extern uint16 ccu6_t12_period_sel;
extern float pwm_period;

extern uint16 ccu6_tctr0_bit7;
extern uint16 ccu6_t12period;
extern uint16 ccu6_t12pr;

extern float ccu6_cc60sr;
extern float ccu6_cc61sr;
extern float ccu6_cc62sr;

typedef union
{
	uint16 dataarray_FOC[NUMBEROF_MESSAGES_FOC];
	TEmo_Focpar_Cfg datastruct_FOC;
}data_FOC_union;
extern data_FOC_union rxtxbuffer;

#endif

