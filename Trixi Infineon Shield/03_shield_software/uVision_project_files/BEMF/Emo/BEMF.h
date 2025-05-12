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

#ifndef BEMF_H
#define BEMF_H
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
#include "tle_device.h"
#include "Emo_BEMF.h"

/*******************************************************************************
**                      Private Macro Definitions                             **
*******************************************************************************/
#define CCU6_MASK_MCMOUTS_SHADOW_HALL  (0x8000u)	//setting this bit in MCMOUTS causes CURH/EXPH to be updated with value from shadow bits CURHS/EXPHS
#define CCU6_MASK_MCMOUTS_SHADOW_OUT   (0x0080u)	//setting this bit in MCMOUTS causes MCMP to be updated with value from shadow bits MCMPS
#define CCU6_MASK_INT_CHE   (0x1000u)			//mask for correct hall event interrupt
#define CCU6_MASK_INT_WHE   (0x2000u)			//mask for wrong hall event interrupt

/*******************************************************************************
**                      Private Function Declarations                         **
*******************************************************************************/
__STATIC_INLINE void BackEMF_InitPatterns(void);		//Initialize Hall-and output pattern for motor startup in sps
__STATIC_INLINE void BackEMF_SwitchtoRamp(void);		//call at motor stop and before motor startup in sps
__STATIC_INLINE void BackEMF_SwitchToHall(void);		//Turn on Hall-Event-Interrupt after Startup
__STATIC_INLINE void BackEMF_SetPatterns(void);		//Set Output-Patterns and Hall-Patterns
__STATIC_INLINE void BackEMF_SetPatterns_Shadow(void);	//Write Output-Patterns and Hall-Patterns into shadow register for next hall event
__STATIC_INLINE void BackEMF_CommutationDelay(void);		//Set Commutation Delay (30°)
__STATIC_INLINE void BackEMF_SpeedCalc(void);		//Calculate Speed from timer difference between Hall-Events


/*******************************************************************************
**                      Global INLINE Function Definitions                    **
*******************************************************************************/
__STATIC_INLINE void BackEMF_InitPatterns(void)
{
  uint32 HallOutPtn;

  CCU6->MODCTR.reg = (uint16)BackEMF_mod[2u + Emo_Ctrl.DirectionIndex];						/* init startup with values for sector 1 (=Table-index 2) */
  HallOutPtn = BackEMF_mcp[0u];								/* init startup with values for sector 1 (=Table-index 2) */
  CCU6->MCMOUTS.reg = (uint16)(HallOutPtn | CCU6_MASK_MCMOUTS_SHADOW_OUT | CCU6_MASK_MCMOUTS_SHADOW_HALL);	/* Set current Hall and output patterns immediately in MCMOUT */
  Emo_Bemf.pattern_bak = (uint16)BackEMF_mod[2u + Emo_Ctrl.DirectionIndex];
}

//Initialize Starting Position and desired direction for ramp up	(in sps)
__STATIC_INLINE void BackEMF_InitPatterns_Start0(void)
{
  uint32 HallOutPtn;

  CCU6->MODCTR.reg = (uint16)BackEMF_mod[2u + Emo_Ctrl.DirectionIndex];						/* init startup with values for sector 1 (=Table-index 2) */
  HallOutPtn = BackEMF_mcp[2u + Emo_Ctrl.DirectionIndex];								/* init startup with values for sector 1 (=Table-index 2) */
  CCU6->MCMOUTS.reg = (uint16)(HallOutPtn | CCU6_MASK_MCMOUTS_SHADOW_OUT | CCU6_MASK_MCMOUTS_SHADOW_HALL);	/* Set current Hall and output patterns immediately in MCMOUT */
  Emo_Bemf.pattern_bak = (uint16)BackEMF_mod[2u + Emo_Ctrl.DirectionIndex];
}

//Initialize Starting Position and desired direction for ramp up	(in sps)
__STATIC_INLINE void BackEMF_InitPatterns_SpeedTest2(void)
{
  uint32 HallOutPtn;
  uint32 CurrentHallPtn;

  CCU6->MODCTR.reg = (uint16)BackEMF_mod[(Emo_Bemf.bemfc_ctrl_sts_bak_old & 7) + Emo_Ctrl.DirectionIndex];						/* init startup with values for sector 1 (=Table-index 2) */
  HallOutPtn = BackEMF_mcp[(Emo_Bemf.bemfc_ctrl_sts_bak_old & 7) + Emo_Ctrl.DirectionIndex];								/* init startup with values for sector 1 (=Table-index 2) */
  HallOutPtn = ((HallOutPtn & 0x700)<<3)|(HallOutPtn & 0x7ff);
  CCU6->MCMOUTS.reg = (uint16)(HallOutPtn | CCU6_MASK_MCMOUTS_SHADOW_OUT | CCU6_MASK_MCMOUTS_SHADOW_HALL);	/* Set current Hall and output patterns immediately in MCMOUT */
  /* Current Hall pattern = expected Hall pattern from last sector*/
  CurrentHallPtn = (CCU6->MCMOUT.reg >> 8u) & 0x7u;
  /* Prepare next Hall patterns in shadow register (required for smooth switch from startup to hall-event-control)	*/
  CCU6->MCMOUTS.reg = BackEMF_mcp[CurrentHallPtn + Emo_Ctrl.DirectionIndex];
}

__STATIC_INLINE void BackEMF_SwitchtoRamp(void)				//call before motor startup / at motor stop
{
  CCU6->T12MSEL.bit.HSYNC = 3u;		//hall sampling triggered by HW sources is turned off
  CCU6->MCMCTR.bit.SWSEL = 0u;				//no trigger request for shadowtransfer of MCMPS to MCMP wil be generated
  CCU6->IEN.reg = CCU6->IEN.reg & (~(CCU6_MASK_INT_CHE | CCU6_MASK_INT_WHE));			/* disable interrupts for Hall events 	*/
}


__STATIC_INLINE void BackEMF_SwitchToHall(void)
{
  CCU6->T12MSEL.bit.HSYNC = 2u;					//T13 Period Match activates HCRDY (evaluation of Hall inputs turns on)
  CCU6->MCMCTR.bit.SWSEL = 1u;					//Correct Hall Pattern (can only be detected after T13 period match) causes shadowtransfer MCMPS --> MCMP
  CCU6->ISR.reg = CCU6_MASK_INT_CHE | CCU6_MASK_INT_WHE;							  	/* Clear status bits for Hall events	*/
  CCU6->IEN.reg = CCU6->IEN.reg | CCU6_MASK_INT_CHE | CCU6_MASK_INT_WHE;				/* enable interrupts for Hall events 	*/
}; /* End of Ccu6_SwitchToHallBemf */


//Sets the Hall and output patterns during ramp-up (call everytime ramp has passed 60°).
__STATIC_INLINE void BackEMF_SetPatterns(void)
{
  uint32 CurrentHallPtn;
  uint32 ExpHallPtn;
  uint32 HallOutPtn;

  /* Current Hall pattern = expected Hall pattern from last sector*/
  CurrentHallPtn = (CCU6->MCMOUT.reg >> 8u) & 0x7u;

  /* enable modulation of PWM-Channels for current sector in CCU60_MODCTR from table */
  CCU6->MODCTR.reg = BackEMF_mod[CurrentHallPtn + Emo_Ctrl.DirectionIndex];

  /* Immediately set Hall patterns and output pattern from table */
  HallOutPtn = BackEMF_mcp[CurrentHallPtn + Emo_Ctrl.DirectionIndex];
  CCU6->MCMOUTS.reg = (uint16)(HallOutPtn | CCU6_MASK_MCMOUTS_SHADOW_OUT | CCU6_MASK_MCMOUTS_SHADOW_HALL);

  /* Prepare next Hall patterns in shadow register (required for smooth switch from startup to hall-event-control)	*/
  ExpHallPtn = (BackEMF_mcp[CurrentHallPtn + Emo_Ctrl.DirectionIndex] >> 8u) & 0x7u;
  CCU6->MCMOUTS.reg = BackEMF_mcp[ExpHallPtn + Emo_Ctrl.DirectionIndex];
} /* End of BackEMF_SetPatterns */

/* sets next hall pattern for shadow transfer when motor is running */
__STATIC_INLINE void BackEMF_SetPatterns_Shadow(void)
{
  uint32 CurrentHallPtn;
  uint32 ExpHallPtn;

  /* read current Hall pattern */
  CurrentHallPtn = (CCU6->MCMOUT.reg >> 11u) & 0x7u;

  /* enable modulation of PWM-Channels for current sector in CCU60_MODCTR from table */
  CCU6->MODCTR.reg = BackEMF_mod[CurrentHallPtn + Emo_Ctrl.DirectionIndex];

  /* read expected Hall pattern */
  ExpHallPtn = (CCU6->MCMOUT.reg >> 8u) & 0x7u;

  /* Prepare next Hall patterns and output pattern - shadowtransfer will happen at next hall event */
  CCU6->MCMOUTS.reg = BackEMF_mcp[ExpHallPtn + Emo_Ctrl.DirectionIndex];
}

__STATIC_INLINE void BackEMF_CommutationDelaywithoutT13(void)					//execute in every Hall-Event-Interrupt and every 60° during startup!
{
  uint32 difftmw;

  Emo_Ctrl.NewTime = GPT12E_T3_Value_Get(); 						/* Read T3 immediately to get time */
  Emo_Ctrl.DiffTime = Emo_Ctrl.NewTime - Emo_Ctrl.OldTime;		/* Calculate Time Difference to last Hall Event	*/
  Emo_Ctrl.OldTime = Emo_Ctrl.NewTime;						/* Remember Time for next Hall Event */
  Emo_Ctrl.T3TimeoutCounter = 0;

  Emo_Ctrl.DiffTimePtr = (Emo_Ctrl.DiffTimePtr + 1) & 0x3;
  Emo_Ctrl.DiffTimeRsp[Emo_Ctrl.DiffTimePtr] = Emo_Ctrl.DiffTime;

  difftmw = Emo_Ctrl.DiffTimeRsp[0] + Emo_Ctrl.DiffTimeRsp[1] + Emo_Ctrl.DiffTimeRsp[2] + Emo_Ctrl.DiffTimeRsp[3];
  Emo_Ctrl.DiffTimeMw = difftmw >> 2;
}



__STATIC_INLINE void BackEMF_CommutationDelay(void)					//execute in every Hall-Event-Interrupt and every 60° during startup!
{
  sint16 CommDelayTime;
  BackEMF_CommutationDelaywithoutT13();

  CommDelayTime =  Emo_Ctrl.DiffTimeMw;					 /* Get half Hall difference time */
  if (CommDelayTime < 1000)
  {
    CommDelayTime=1000;
  }
  CCU6_LoadPeriodRegister_T13_Tick(CommDelayTime - 500);/* Set Commutation Delay for next Hall-Event */
  CCU6_EnableST_T13();
} /* End of BackEMF_CommutationDelay */

__STATIC_INLINE void BackEMF_CommutationDelayhalf(void)					//execute in every Hall-Event-Interrupt and every 60° during startup!
{
  sint16 CommDelayTime;

  BackEMF_CommutationDelaywithoutT13();

  CommDelayTime = Emo_Ctrl.DiffTimeMw >> 1u;					 /* Get half Hall difference time */
  CCU6_LoadPeriodRegister_T13_Tick(CommDelayTime);/* Set Commutation Delay for next Hall-Event */
  CCU6_EnableST_T13();
} /* End of BackEMF_CommutationDelay */

#endif /* BEMF_H */


