/*
 * Global variables needed for changing HALL parameters with arduino
 * initialized with standard values from defines
 */

#include "ccu6_defines.h"               // Infineon::Device:ConfigWizard
#include "types.h"
#include "bchall_defines.h" 
#include "globvars_HALL.h"

const uint8 indices_16bit[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
const uint8 indices_16bit_size = (uint8) (sizeof(indices_16bit)/sizeof(indices_16bit[0]));

/* for changing HALL_PWM_FREQ  */
float ccu6_t12_freq = CCU6_T12_FREQ;
uint16 ccu6_t12_period_sel = CCU6_T12_PERIOD_SEL;
float pwm_period = BCHALL_PWM_PERIOD;

uint16 ccu6_tctr0_bit7 = (CCU6_TCTR0 & 0x0080) >> 7;
uint16 ccu6_t12period = CCU6_T12PERIOD;
uint16 ccu6_t12pr = CCU6_T12PR;

float ccu6_cc60sr = CCU6_CC60SR;
float ccu6_cc61sr = CCU6_CC61SR;
float ccu6_cc62sr = CCU6_CC62SR;

data_HALL_union rxtxbuffer = {0};
