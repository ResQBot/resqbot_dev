/*
 * Global variables needed for changing BEMF parameters with arduino
 * initialized with standard values from defines
 */
 

#include "ccu6_defines.h"               // Infineon::Device:ConfigWizard
#include "types.h"
#include "bemf_defines.h"
#include "globvars_BEMF.h"


const uint8 indices_16bit[] = {27, 28, 6, 2, 3, 17, 16};
const uint8 indices_16bit_size = (uint8) (sizeof(indices_16bit)/sizeof(indices_16bit[0]));

/* for changing BEMF_PWM_FREQ  */
float ccu6_t12_freq = CCU6_T12_FREQ;
uint16 ccu6_t12_period_sel = CCU6_T12_PERIOD_SEL;
float pwm_period = BEMF_PWM_PERIOD;

uint16 ccu6_tctr0_bit7 = (CCU6_TCTR0 & 0x0080) >> 7;
uint16 ccu6_t12period = CCU6_T12PERIOD;
uint16 ccu6_t12pr = CCU6_T12PR;

float ccu6_cc60sr = CCU6_CC60SR;
float ccu6_cc61sr = CCU6_CC61SR;
float ccu6_cc62sr = CCU6_CC62SR;

/* for changing BEMF_START_SPEED_PWM_MIN_OFFSET and BEMF_END_SPEED_PWM_MIN */
float bemfEndSpeedPwmMin = BEMF_END_SPEED_PWM_MIN;

/* for saving BEMF_SPIKE_FILT and BEMF_BLANK_FILT */
uint16 bemf_spike_filt = 0;
uint16 bemf_blank_filt = 0;

data_BEMF_union rxtxbuffer = {0};

