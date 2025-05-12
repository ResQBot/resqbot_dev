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
**                      Includes                                              **
*******************************************************************************/
#include "EmoCcu.h"

/******************************************************************************
**                      Private Macro Definitions                            **
*******************************************************************************/


/*******************************************************************************
**                      Global Constant Definitions to be changed             **
*******************************************************************************/
TEmoCcu_Cfg EmoCcu_Cfg =
{
  /* HallOutPtns[16] - Hall and output patterns */
  { 
		(uint16)0, 
		(uint16)0, 
		(uint16)0, 
		(uint16)0, 
		(uint16)0, 
		(uint16)0, 
		(uint16)0, 
		(uint16)0, 
		(uint16)0, 
		(uint16)0, 
		(uint16)0, 
		(uint16)0, 
		(uint16)0, 
		(uint16)0, 
		(uint16)0, 
		(uint16)0, 
  },
  /* T12Moden - T12 modulation enable patterns */
  {
		(uint8)0, 
		(uint8)0, 
		(uint8)0, 
		(uint8)0, 
		(uint8)0, 
		(uint8)0, 
		(uint8)0, 
		(uint8)0, 
		(uint8)0, 
		(uint8)0, 
		(uint8)0, 
		(uint8)0, 
		(uint8)0, 
		(uint8)0, 
		(uint8)0, 
		(uint8)0
  },
}; /* End of EmoCcu_Cfg */


TEmo_Hallpar_Cfg Emo_Hallpar_Cfg =
{
	/* PWM_Frequency */
  (float)	BCHALL_PWM_FREQ,
	/* PolePair */	
	(uint16) BCHALL_POLE_PAIRS,
	/* initDutyCycle */	
	(uint16) BCHALL_INIT_DUTY,
	/* inputA */	
	(uint16) BCHALL_INPUT_A,
	/* inputB */	
	(uint16) BCHALL_INPUT_B,
	/* inputC */	
	(uint16) BCHALL_INPUT_C,
	/* offset60degEn */	
	(uint16) BCHALL_OFFSET_60DEGREE_EN,
	/* angleDelayEn */	
	(uint16) BCHALL_ANGLE_DELAY_EN,
	/* delayAngle */	
	(uint16) BCHALL_DELAY_ANGLE,
	/* delayMinspeed */	
	(uint16) BCHALL_DELAY_MINSPEED,
	/* SpeedPi_Kp */	
	(uint16) BCHALL_SPEED_KP,
	/* SpeedPi_Ki */	
	(uint16) BCHALL_SPEED_KI,
	/* speedIMin */	
	(float)	BCHALL_SPEED_IMIN,
	/* speedIMax */	
	(float)BCHALL_SPEED_IMAX,
	/* speedPiMin */	
	(float)	BCHALL_SPEED_PIMIN,
	/* speedPiMax */	
	(float)	BCHALL_SPEED_PIMAX,
};

const TEmo_Hallpar_Cfg dataset0 =
{
	/* PWM_Frequency */
  (float)	BCHALL_PWM_FREQ,
	/* PolePair */	
	(uint16) BCHALL_POLE_PAIRS,
	/* initDutyCycle */	
	(uint16) BCHALL_INIT_DUTY,
	/* inputA */	
	(uint16) BCHALL_INPUT_A,
	/* inputB */	
	(uint16) BCHALL_INPUT_B,
	/* inputC */	
	(uint16) BCHALL_INPUT_C,
	/* offset60degEn */	
	(uint16) BCHALL_OFFSET_60DEGREE_EN,
	/* angleDelayEn */	
	(uint16) BCHALL_ANGLE_DELAY_EN,
	/* delayAngle */	
	(uint16) BCHALL_DELAY_ANGLE,
	/* delayMinspeed */	
	(uint16) BCHALL_DELAY_MINSPEED,
	/* SpeedPi_Kp */	
	(uint16) BCHALL_SPEED_KP,
	/* SpeedPi_Ki */	
	(uint16) BCHALL_SPEED_KI,
	/* speedIMin */	
	(float)	BCHALL_SPEED_IMIN,
	/* speedIMax */	
	(float)BCHALL_SPEED_IMAX,
	/* speedPiMin */	
	(float)	BCHALL_SPEED_PIMIN,
	/* speedPiMax */	
	(float)	BCHALL_SPEED_PIMAX,
};

const TEmo_Hallpar_Cfg dataset1 =
{
	/* PWM_Frequency */
  (float)	0,
	/* PolePair */	
	(uint16) 0,
	/* initDutyCycle */	
	(uint16) 0,
	/* inputA */	
	(uint16) 0,
	/* inputB */	
	(uint16) 0,
	/* inputC */	
	(uint16) 0,
	/* offset60degEn */	
	(uint16) 0,
	/* angleDelayEn */	
	(uint16) 0,
	/* delayAngle */	
	(uint16) 0,
	/* delayMinspeed */	
	(uint16) 0,
	/* SpeedPi_Kp */	
	(uint16) 0,
	/* SpeedPi_Ki */	
	(uint16) 0,
	/* speedIMin */	
	(float)	0,
	/* speedIMax */	
	(float)0,
	/* speedPiMin */	
	(float)	0,
	/* speedPiMax */	
	(float)	0,
};

const TEmo_Hallpar_Cfg dataset2 =
{
	/* PWM_Frequency */
  (float)	0,
	/* PolePair */	
	(uint16) 0,
	/* initDutyCycle */	
	(uint16) 0,
	/* inputA */	
	(uint16) 0,
	/* inputB */	
	(uint16) 0,
	/* inputC */	
	(uint16) 0,
	/* offset60degEn */	
	(uint16) 0,
	/* angleDelayEn */	
	(uint16) 0,
	/* delayAngle */	
	(uint16) 0,
	/* delayMinspeed */	
	(uint16) 0,
	/* SpeedPi_Kp */	
	(uint16) 0,
	/* SpeedPi_Ki */	
	(uint16) 0,
	/* speedIMin */	
	(float)	0,
	/* speedIMax */	
	(float)0,
	/* speedPiMin */	
	(float)	0,
	/* speedPiMax */	
	(float)	0,
};

const TEmo_Hallpar_Cfg dataset3 =
{
	/* PWM_Frequency */
  (float)	0,
	/* PolePair */	
	(uint16) 0,
	/* initDutyCycle */	
	(uint16) 0,
	/* inputA */	
	(uint16) 0,
	/* inputB */	
	(uint16) 0,
	/* inputC */	
	(uint16) 0,
	/* offset60degEn */	
	(uint16) 0,
	/* angleDelayEn */	
	(uint16) 0,
	/* delayAngle */	
	(uint16) 0,
	/* delayMinspeed */	
	(uint16) 0,
	/* SpeedPi_Kp */	
	(uint16) 0,
	/* SpeedPi_Ki */	
	(uint16) 0,
	/* speedIMin */	
	(float)	0,
	/* speedIMax */	
	(float)0,
	/* speedPiMin */	
	(float)	0,
	/* speedPiMax */	
	(float)	0,
};

const TEmo_Hallpar_Cfg customdataset0 =
{
	/* PWM_Frequency */
  (float)	0,
	/* PolePair */	
	(uint16) 0,
	/* initDutyCycle */	
	(uint16) 0,
	/* inputA */	
	(uint16) 0,
	/* inputB */	
	(uint16) 0,
	/* inputC */	
	(uint16) 0,
	/* offset60degEn */	
	(uint16) 0,
	/* angleDelayEn */	
	(uint16) 0,
	/* delayAngle */	
	(uint16) 0,
	/* delayMinspeed */	
	(uint16) 0,
	/* SpeedPi_Kp */	
	(uint16) 0,
	/* SpeedPi_Ki */	
	(uint16) 0,
	/* speedIMin */	
	(float)	0,
	/* speedIMax */	
	(float)0,
	/* speedPiMin */	
	(float)	0,
	/* speedPiMax */	
	(float)	0,
};

const TEmo_Hallpar_Cfg customdataset1 =
{
	/* PWM_Frequency */
  (float)	0,
	/* PolePair */	
	(uint16) 0,
	/* initDutyCycle */	
	(uint16) 0,
	/* inputA */	
	(uint16) 0,
	/* inputB */	
	(uint16) 0,
	/* inputC */	
	(uint16) 0,
	/* offset60degEn */	
	(uint16) 0,
	/* angleDelayEn */	
	(uint16) 0,
	/* delayAngle */	
	(uint16) 0,
	/* delayMinspeed */	
	(uint16) 0,
	/* SpeedPi_Kp */	
	(uint16) 0,
	/* SpeedPi_Ki */	
	(uint16) 0,
	/* speedIMin */	
	(float)	0,
	/* speedIMax */	
	(float)0,
	/* speedPiMin */	
	(float)	0,
	/* speedPiMax */	
	(float)	0,
};

const TEmo_Hallpar_Cfg customdataset2 =
{
	/* PWM_Frequency */
  (float)	0,
	/* PolePair */	
	(uint16) 0,
	/* initDutyCycle */	
	(uint16) 0,
	/* inputA */	
	(uint16) 0,
	/* inputB */	
	(uint16) 0,
	/* inputC */	
	(uint16) 0,
	/* offset60degEn */	
	(uint16) 0,
	/* angleDelayEn */	
	(uint16) 0,
	/* delayAngle */	
	(uint16) 0,
	/* delayMinspeed */	
	(uint16) 0,
	/* SpeedPi_Kp */	
	(uint16) 0,
	/* SpeedPi_Ki */	
	(uint16) 0,
	/* speedIMin */	
	(float)	0,
	/* speedIMax */	
	(float)0,
	/* speedPiMin */	
	(float)	0,
	/* speedPiMax */	
	(float)	0,
};

const TEmo_Hallpar_Cfg customdataset3 =
{
	/* PWM_Frequency */
  (float)	0,
	/* PolePair */	
	(uint16) 0,
	/* initDutyCycle */	
	(uint16) 0,
	/* inputA */	
	(uint16) 0,
	/* inputB */	
	(uint16) 0,
	/* inputC */	
	(uint16) 0,
	/* offset60degEn */	
	(uint16) 0,
	/* angleDelayEn */	
	(uint16) 0,
	/* delayAngle */	
	(uint16) 0,
	/* delayMinspeed */	
	(uint16) 0,
	/* SpeedPi_Kp */	
	(uint16) 0,
	/* SpeedPi_Ki */	
	(uint16) 0,
	/* speedIMin */	
	(float)	0,
	/* speedIMax */	
	(float)0,
	/* speedPiMin */	
	(float)	0,
	/* speedPiMax */	
	(float)	0,
};

