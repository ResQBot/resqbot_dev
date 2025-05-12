/**
 * @cond
 ***********************************************************************************************************************
 *
 * Copyright (c) 2015, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************************************************************/
/*******************************************************************************
**                      Author(s) Identity                                    **
********************************************************************************
**                                                                            **
** Initials     Name                                                          **
** ---------------------------------------------------------------------------**
** ISLE         ISLE Ilmenau                                                  **
**                                                                            **
**                                                                            **
*******************************************************************************/

/*******************************************************************************
**                      Revision Control History                              **
*******************************************************************************/
/*
 * V0.1.0: 2016-02-12, ISLE:   Initial version
 */

/*******************************************************************************
**                      Includes                                              **
*******************************************************************************/
#include "Emo_BEMF.h"
#include "bemf_defines.h"
/*******************************************************************************
**                      Global Constant Definitions to be changed             **
*******************************************************************************/
/* standard configuration */
TEmo_Bemfpar_Cfg Emo_Bemfpar_Cfg =
{
    /* PWM_Frequency */
    (float) BEMF_PWM_FREQ,
    /* Speedcontroller Kp */
    (uint16) BEMF_SPEED_KP,
    /* Speedcontroller Ki */
    (uint16) BEMF_SPEED_KI,
    /* Time constant for speed filter in s*/
    (float) BEMF_SPEED_FILT_TIME,
    /* Polpaarzahl  */
    /* Range: 1..32 */
    (uint16) BEMF_POLE_PAIRS,
    /*Time for Speed zero in s */
    (float) BEMF_ZERO_VEC_TIME,
    /* Max Speed for Start in 1/rpm */
    (float) BEMF_END_START_SPEED,
    /* Start Speed SlewRate in 1/rpm/s */
    (float) BEMF_START_ACCEL,
    /*  Speed SlewRate in 1/rpm/s */
    (float) BEMF_RUN_ACCEL,
    /*Enable Start width Frequenz=0 */
    (uint16) BEMF_START_FREQ_ZERO,
    /*Speed Test Enable */
    (uint16)BEMF_SPEED_TEST_ENABLE,
    /*Speed level switch on */
    (float) BEMF_SWITCH_ON_SPEED,
    /*Speed pwm start min */
    (float) BEMF_START_SPEED_PWM_MIN,
    /*Speed pwm start min offset*/
    (float) BEMF_START_SPEED_PWM_MIN_OFFSET,
    /*Speed pwm begin run min              */
    (float) BEMF_RUNBEGIN_SPEED_PWM_MIN,
    /*Speed pwm run min */
    (float) BEMF_RUN_SPEED_PWM_MIN,
};/* End of Emo_Bemfpar_Cfg */


/* dataset0 */
const Tdatastruct_BEMF dataset0 = 
{
	{
    /* PWM_Frequency */
    (float) BEMF_PWM_FREQ,
    /* Speedcontroller Kp */
    (uint16) BEMF_SPEED_KP,
    /* Speedcontroller Ki */
    (uint16) BEMF_SPEED_KI,
    /* Time constant for speed filter in s*/
    (float) BEMF_SPEED_FILT_TIME,
    /* Polpaarzahl  */
    /* Range: 1..32 */
    (uint16)BEMF_POLE_PAIRS,
    /*Time for Speed zero in s */
    (float) BEMF_ZERO_VEC_TIME,
    /* Max Speed for Start in 1/rpm */
    (float) BEMF_END_START_SPEED,
    /* Start Speed SlewRate in 1/rpm/s */
    (float) BEMF_START_ACCEL,
    /*  Speed SlewRate in 1/rpm/s */
    (float) BEMF_RUN_ACCEL,
    /*Enable Start width Frequenz=0 */
    (uint16) BEMF_START_FREQ_ZERO,
    /*Speed Test Enable */
    (uint16)BEMF_SPEED_TEST_ENABLE,
    /*Speed level switch on */
    (float) BEMF_SWITCH_ON_SPEED,
    /*Speed pwm start min */
    (float) BEMF_START_SPEED_PWM_MIN,
    /*Speed pwm start min offset*/
    (float) BEMF_START_SPEED_PWM_MIN_OFFSET,
    /*Speed pwm begin run min              */
    (float) BEMF_RUNBEGIN_SPEED_PWM_MIN,
    /*Speed pwm run min */
    (float) BEMF_RUN_SPEED_PWM_MIN,
	},
	0,
	0
};
	

const Tdatastruct_BEMF dataset1 = 
{
	{
    /* PWM_Frequency */
    (float) 0,
    /* Speedcontroller Kp */
    (uint16) 0,
    /* Speedcontroller Ki */
    (uint16) 0,
    /* Time constant for speed filter in s*/
    (float) 0,
    /* Polpaarzahl  */
    /* Range: 1..32 */
    (uint16)0,
    /*Time for Speed zero in s */
    (float) 0,
    /* Max Speed for Start in 1/rpm */
    (float) 0,
    /* Start Speed SlewRate in 1/rpm/s */
    (float) 0,
    /*  Speed SlewRate in 1/rpm/s */
    (float) 0,
    /*Enable Start width Frequenz=0 */
    (uint16) 0,
    /*Speed Test Enable */
    (uint16)0,
    /*Speed level switch on */
    (float) 0,
    /*Speed pwm start min */
    (float) 0,
    /*Speed pwm start min offset*/
    (float) 0,
    /*Speed pwm begin run min              */
    (float) 0,
    /*Speed pwm run min */
    (float) 0,
	},
	0,
	0
};


const Tdatastruct_BEMF dataset2 = 
{
	{
    /* PWM_Frequency */
    (float) 0,
    /* Speedcontroller Kp */
    (uint16) 0,
    /* Speedcontroller Ki */
    (uint16) 0,
    /* Time constant for speed filter in s*/
    (float) 0,
    /* Polpaarzahl  */
    /* Range: 1..32 */
    (uint16)0,
    /*Time for Speed zero in s */
    (float) 0,
    /* Max Speed for Start in 1/rpm */
    (float) 0,
    /* Start Speed SlewRate in 1/rpm/s */
    (float) 0,
    /*  Speed SlewRate in 1/rpm/s */
    (float) 0,
    /*Enable Start width Frequenz=0 */
    (uint16) 0,
    /*Speed Test Enable */
    (uint16)0,
    /*Speed level switch on */
    (float) 0,
    /*Speed pwm start min */
    (float) 0,
    /*Speed pwm start min offset*/
    (float) 0,
    /*Speed pwm begin run min              */
    (float) 0,
    /*Speed pwm run min */
    (float) 0,
	},
	0,
	0
};


const Tdatastruct_BEMF dataset3 = 
{
	{
    /* PWM_Frequency */
    (float) 0,
    /* Speedcontroller Kp */
    (uint16) 0,
    /* Speedcontroller Ki */
    (uint16) 0,
    /* Time constant for speed filter in s*/
    (float) 0,
    /* Polpaarzahl  */
    /* Range: 1..32 */
    (uint16)0,
    /*Time for Speed zero in s */
    (float) 0,
    /* Max Speed for Start in 1/rpm */
    (float) 0,
    /* Start Speed SlewRate in 1/rpm/s */
    (float) 0,
    /*  Speed SlewRate in 1/rpm/s */
    (float) 0,
    /*Enable Start width Frequenz=0 */
    (uint16) 0,
    /*Speed Test Enable */
    (uint16)0,
    /*Speed level switch on */
    (float) 0,
    /*Speed pwm start min */
    (float) 0,
    /*Speed pwm start min offset*/
    (float) 0,
    /*Speed pwm begin run min              */
    (float) 0,
    /*Speed pwm run min */
    (float) 0,
	},
	0,
	0
};

/* custom dataset0 */
const Tdatastruct_BEMF customdataset0 =
{
	{
    /* PWM_Frequency */
    (float) 0,
    /* Speedcontroller Kp */
    (uint16) 0,
    /* Speedcontroller Ki */
    (uint16) 0,
    /* Time constant for speed filter in s*/
    (float) 0,
    /* Polpaarzahl  */
    /* Range: 1..32 */
    (uint16)0,
    /*Time for Speed zero in s */
    (float) 0,
    /* Max Speed for Start in 1/rpm */
    (float) 0,
    /* Start Speed SlewRate in 1/rpm/s */
    (float) 0,
    /*  Speed SlewRate in 1/rpm/s */
    (float) 0,
    /*Enable Start width Frequenz=0 */
    (uint16) 0,
    /*Speed Test Enable */
    (uint16)0,
    /*Speed level switch on */
    (float) 0,
    /*Speed pwm start min */
    (float) 0,
    /*Speed pwm start min offset*/
    (float) 0,
    /*Speed pwm begin run min              */
    (float) 0,
    /*Speed pwm run min */
    (float) 0,
	},
	0,
	0
};


/* custom dataset1 */
const Tdatastruct_BEMF customdataset1 =
{
	{
    /* PWM_Frequency */
    (float) 0,
    /* Speedcontroller Kp */
    (uint16) 0,
    /* Speedcontroller Ki */
    (uint16) 0,
    /* Time constant for speed filter in s*/
    (float) 0,
    /* Polpaarzahl  */
    /* Range: 1..32 */
    (uint16)0,
    /*Time for Speed zero in s */
    (float) 0,
    /* Max Speed for Start in 1/rpm */
    (float) 0,
    /* Start Speed SlewRate in 1/rpm/s */
    (float) 0,
    /*  Speed SlewRate in 1/rpm/s */
    (float) 0,
    /*Enable Start width Frequenz=0 */
    (uint16) 0,
    /*Speed Test Enable */
    (uint16)0,
    /*Speed level switch on */
    (float) 0,
    /*Speed pwm start min */
    (float) 0,
    /*Speed pwm start min offset*/
    (float) 0,
    /*Speed pwm begin run min              */
    (float) 0,
    /*Speed pwm run min */
    (float) 0,
	},
	0,
	0
};


/* custom dataset2 */
const Tdatastruct_BEMF customdataset2 =
{
	{
    /* PWM_Frequency */
    (float) 0,
    /* Speedcontroller Kp */
    (uint16) 0,
    /* Speedcontroller Ki */
    (uint16) 0,
    /* Time constant for speed filter in s*/
    (float) 0,
    /* Polpaarzahl  */
    /* Range: 1..32 */
    (uint16)0,
    /*Time for Speed zero in s */
    (float) 0,
    /* Max Speed for Start in 1/rpm */
    (float) 0,
    /* Start Speed SlewRate in 1/rpm/s */
    (float) 0,
    /*  Speed SlewRate in 1/rpm/s */
    (float) 0,
    /*Enable Start width Frequenz=0 */
    (uint16) 0,
    /*Speed Test Enable */
    (uint16)0,
    /*Speed level switch on */
    (float) 0,
    /*Speed pwm start min */
    (float) 0,
    /*Speed pwm start min offset*/
    (float) 0,
    /*Speed pwm begin run min              */
    (float) 0,
    /*Speed pwm run min */
    (float) 0,
	},
	0,
	0
};


/* custom dataset3 */
const Tdatastruct_BEMF customdataset3 =
{
	{
    /* PWM_Frequency */
    (float) 0,
    /* Speedcontroller Kp */
    (uint16) 0,
    /* Speedcontroller Ki */
    (uint16) 0,
    /* Time constant for speed filter in s*/
    (float) 0,
    /* Polpaarzahl  */
    /* Range: 1..32 */
    (uint16)0,
    /*Time for Speed zero in s */
    (float) 0,
    /* Max Speed for Start in 1/rpm */
    (float) 0,
    /* Start Speed SlewRate in 1/rpm/s */
    (float) 0,
    /*  Speed SlewRate in 1/rpm/s */
    (float) 0,
    /*Enable Start width Frequenz=0 */
    (uint16) 0,
    /*Speed Test Enable */
    (uint16)0,
    /*Speed level switch on */
    (float) 0,
    /*Speed pwm start min */
    (float) 0,
    /*Speed pwm start min offset*/
    (float) 0,
    /*Speed pwm begin run min              */
    (float) 0,
    /*Speed pwm run min */
    (float) 0,
	},
	0,
	0
};
