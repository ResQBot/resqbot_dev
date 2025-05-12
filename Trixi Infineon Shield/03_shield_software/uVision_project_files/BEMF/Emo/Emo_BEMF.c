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
#include "BEMF.h"
#include "..\..\commonMC.h"

/*******************************************************************************/
/**                      Private Macro Definitions                            **/
/*******************************************************************************/
/* Constants for system interrupt registers SYS_ISCLR, SYS_IS, SYS_IRQ_CTRL */
#define CCU6_MASK_SYS_IS_PHU_ZCLOW (0x00010000u)
#define CCU6_MASK_SYS_IS_PHU_ZCHI  (0x00020000u)
#define CCU6_MASK_SYS_IS_PHV_ZCLOW (0x00040000u)
#define CCU6_MASK_SYS_IS_PHV_ZCHI  (0x00080000u)
#define CCU6_MASK_SYS_IS_PHW_ZCLOW (0x00100000u)
#define CCU6_MASK_SYS_IS_PHW_ZCHI  (0x00200000u)
#define CCU6_MASK_SYS_IS_PH_ALL    (0x003F0000u)

/*******************************************************************************
**                      Private Type Definitions                              **
*******************************************************************************/

/*******************************************************************************
**                      Private Function Declarations                         **
*******************************************************************************/


/*******************************************************************************
**                      Global Variable Definitions                           **
*******************************************************************************/
TEmo_Ctrl Emo_Ctrl;
TEmo_Bemf Emo_Bemf;

/*******************************************************************************
**                      Private Variable Definitions                          **
*******************************************************************************/

/*******************************************************************************
**                      Global Function Definitions                           **
*******************************************************************************/

void BackEMF_Handle_Correct(void)
{
    switch (Emo_Status.MotorState)
    {
        case  EMO_MOTOR_STATE_SPEEDTEST1: break;
        case  EMO_MOTOR_STATE_SPEEDTEST2: break;
        case  EMO_MOTOR_STATE_START0: break;
        case  EMO_MOTOR_STATE_START1: break;

        case  EMO_MOTOR_STATE_START2:
        {
            BackEMF_CommutationDelaywithoutT13();
            Emo_Bemf.bemf_counter--;

            if (Emo_Bemf.bemf_counter == 0) Emo_Status.MotorState = EMO_MOTOR_STATE_START3;
        } break;

        case  EMO_MOTOR_STATE_START3:
        {
            BackEMF_CommutationDelayhalf();
            Emo_Bemf.bemf_counter = EMO_START3_START4_COUNT;
            Emo_Status.MotorState = EMO_MOTOR_STATE_START4;
        } break;

        case  EMO_MOTOR_STATE_START4:
        {
            Emo_Bemf.bemf_counter--;

            if (Emo_Bemf.bemf_counter == 0)
            {
                Emo_Status.MotorState = EMO_MOTOR_STATE_START5;
                BackEMF_CommutationDelayhalf();
                Emo_Bemf.bemf_counter = EMO_START4_START5_COUNT;
                MF->BEMFC_CTRL_STS.reg = MF->BEMFC_CTRL_STS.reg & (~0x0008);    //turn on demag_filter
            }
            else
            {
                BackEMF_CommutationDelaywithoutT13();
            }
        } break;

        case  EMO_MOTOR_STATE_START5:
        {
            BackEMF_CommutationDelayhalf();
            Emo_Bemf.bemf_counter--;

            if (Emo_Bemf.bemf_counter == 0)
            {
                Emo_Status.MotorState = EMO_MOTOR_STATE_START6;
                Emo_Bemf.bemf_counter = EMO_START5_START6_COUNT;
            }
        } break;

        case  EMO_MOTOR_STATE_START6:
        {
            BackEMF_CommutationDelayhalf();
            Emo_Bemf.bemf_counter--;

            if (Emo_Bemf.bemf_counter == 0) Emo_Status.MotorState = EMO_MOTOR_STATE_RUN;
        } break;

        case  EMO_MOTOR_STATE_RUN:
        {
            BackEMF_CommutationDelayhalf();
            Emo_Ctrl.WronghallCounter = 0;
        } break;
    }

    BackEMF_SetPatterns_Shadow();          //write new Hall pattern and Output pattern to shadow registers (update of actual registers with next hall-event)
    Emo_Bemf.difftimeout = Emo_Bemf.timeoutcounter;
    Emo_Bemf.timeoutcounter = 0;
}


void BackEMF_Handle_Wrong(void)
{
    Emo_Ctrl.WronghallCounter++;

    if (Emo_Ctrl.WronghallCounter >= 3) Emo_StopMotor();

    BackEMF_SetPatterns();
    BackEMF_CommutationDelayhalf();
    Emo_Bemf.difftimeout = Emo_Bemf.timeoutcounter;
    Emo_Bemf.timeoutcounter = 0;
}

void Pwm_Handle(void)
{
    uint32 angle6;      /* 6 times start angle  */
    uint16 sector_no;
    uint16 speedpos;
    uint16 i;
    uint16 j;
    uint16 bed;
    static uint16 taskswitchtohall;
    static uint16 counter;

    Emo_Bemf.timeoutcounter++;

    switch (Emo_Status.MotorState)
    {
        case  EMO_MOTOR_STATE_SPEEDTEST1:
        {
            bed = 0;
            i = (MF->BEMFC_CTRL_STS.reg >> 16) & 0x7;
            j = BackEMF_mcp[i + Emo_Ctrl.DirectionIndex];

            if (i != Emo_Bemf.bemfc_ctrl_sts_bak)
            {
                if (i == ((Emo_Bemf.pattern_bak >> 8) & 0x7))
                {
                    counter++;

                    if (counter > 3)
                    {
                        BackEMF_CommutationDelayhalf();
											
                        if (counter > 100) counter = 100;
											
                        bed = 1;
                        Emo_Bemf.difftimeout = Emo_Bemf.timeoutcounter;
                        Emo_Bemf.timeoutcounter = 0;
                    }
                }
                else
                {
                    counter = 0;
                }
            }

            if (Emo_Bemf.timestatus > 250)
            {
                if ((counter > 6) && (abs(Emo_Ctrl.ActSpeed)) > 10)
                {
                    if (bed == 1)
                    {
                        Emo_Bemf.bemfc_ctrl_sts_bak_old = Emo_Bemf.bemfc_ctrl_sts_bak;
                        Emo_Status.MotorState = EMO_MOTOR_STATE_SPEEDTEST2;
                    }
                }
                else
                {
                    Emo_Status.MotorState = EMO_MOTOR_STATE_START0;
                    BackEMF_InitPatterns_Start0();
                }
            }
            else
            {
                if (Emo_Bemf.speedtestenable != 1)
                {
                    Emo_Status.MotorState = EMO_MOTOR_STATE_START0;
                    BackEMF_InitPatterns_Start0();
                }
            }

            Emo_Bemf.pattern_bak = j;
            Emo_Bemf.bemfc_ctrl_sts_bak = i;
        } break;

        case  EMO_MOTOR_STATE_SPEEDTEST2:
        {
            Emo_Ctrl.RefSpeedSlopeMem = ((uint32)Emo_Ctrl.ActSpeed) << 16;
            Emo_Ctrl.SpeedPi.IOut = Emo_Ctrl.StartSpeedPwmMin;
            i = __ssat(Mat_FixMulScale(Emo_Ctrl.StartSpeedPwmMin, Emo_Bemf.DcFactor, 2), 15u);

            if (i > Emo_Ctrl.SpeedPi.PiMax) i = Emo_Ctrl.SpeedPi.PiMax;

            Emo_Ctrl.vout = Emo_Ctrl.Period - i;
            CCU6_LoadShadowRegister_CC60(Emo_Ctrl.vout);
            CCU6_LoadShadowRegister_CC61(Emo_Ctrl.vout);
            CCU6_LoadShadowRegister_CC62(Emo_Ctrl.vout);
            CCU6_EnableST_T12();
            MF->BEMFC_CTRL_STS.reg = MF->BEMFC_CTRL_STS.reg & (~0x0008);    //turn on demag_filter
            BackEMF_InitPatterns_SpeedTest2();
            BackEMF_SwitchToHall();
            Emo_Status.MotorState = EMO_MOTOR_STATE_RUN;
        } break;

        case  EMO_MOTOR_STATE_START0:
        {
            Emo_Bemf.StartAngle += Emo_Bemf.StartFrequencySlope;
            angle6 = 6 * (uint32)Emo_Bemf.StartAngle;
            sector_no = (angle6 >> 16);

            if (sector_no != Emo_Bemf.sector_no_sp)
                /* every 60° of motor angle, force commutation  */
            {
                BackEMF_CommutationDelay();             //calculate commutation delay, for smooth change to hall event control
                BackEMF_SetPatterns();                  //immediately switch patterns to next sector
                Emo_Bemf.difftimeout = Emo_Bemf.timeoutcounter;
                Emo_Bemf.timeoutcounter = 0;
            }

            Emo_Bemf.sector_no_sp = sector_no;
        } break;

        case  EMO_MOTOR_STATE_START1:
        {
            Emo_Bemf.StartAngle += Emo_Bemf.StartFrequencySlope;
            angle6 = 6 * (uint32)Emo_Bemf.StartAngle;
            sector_no = (angle6 >> 16);

            if (sector_no != Emo_Bemf.sector_no_sp)
                /* every 60° of motor angle, force commutation  */
            {
                BackEMF_CommutationDelay();             //calculate commutation delay, for smooth change to hall event control
                BackEMF_SetPatterns();                  //immediately switch patterns to next sector
                BackEMF_SwitchToHall();             //turn on Hall-Event-Interrupt
                taskswitchtohall = 1;
                Emo_Status.MotorState = EMO_MOTOR_STATE_START2;
                Emo_Bemf.bemf_counter = EMO_START1_START2_COUNT;
                Emo_Ctrl.WronghallCounter = 0;
                Emo_Bemf.difftimeout = Emo_Bemf.timeoutcounter;
                Emo_Bemf.timeoutcounter = 0;
            }

            Emo_Bemf.sector_no_sp = sector_no;
        } break;

        case  EMO_MOTOR_STATE_START2:
        {
            if (taskswitchtohall == 1) taskswitchtohall = 0;
        } break;
	
        case  EMO_MOTOR_STATE_START3:
        case  EMO_MOTOR_STATE_START4:
        case  EMO_MOTOR_STATE_START5:
        case  EMO_MOTOR_STATE_START6:
        case  EMO_MOTOR_STATE_RUN:
						if (Emo_Bemf.timeoutcounter > (3 * Emo_Bemf.difftimeout)) Emo_StopMotor();
						break;
    }

    Emo_Ctrl.T3TimeoutCounter++;

    if (Emo_Ctrl.T3TimeoutCounter > 5000)
    {
        Emo_Ctrl.T3TimeoutCounter = 0;
        Emo_Ctrl.Speed = 0;
    }

    /* Calculate actual speed */
    Emo_Ctrl.DiffTimept1 = Mat_ExeLp_without_min_max(&Emo_Ctrl.DiffTimeLp, Emo_Ctrl.DiffTime);
    speedpos = Emo_Ctrl.SpeedConst / Emo_Ctrl.DiffTimept1 / Emo_Bemf.PolePair;

		
		
    if (Emo_Ctrl.RefSpeed > 0)
    {
        Emo_Ctrl.Speed = speedpos;
        if (Emo_Ctrl.Speed < 0) Emo_Ctrl.Speed = 0;
    }
    else
    {
        Emo_Ctrl.Speed = -speedpos;                                 /* Calculate actual speed */
        if (Emo_Ctrl.Speed > 0) Emo_Ctrl.Speed = 0;
    }

    if (Emo_Status.MotorState < EMO_MOTOR_STATE_SPEEDTEST1)
    {
        Emo_Ctrl.Speed = 0;
    }
		
    if ((counter <= 6) && ((Emo_Status.MotorState == EMO_MOTOR_STATE_SPEEDTEST1) 
															|| (Emo_Status.MotorState == EMO_MOTOR_STATE_START0)))
    {
        Emo_Ctrl.Speed = 0;
    }
		
    Emo_Ctrl.ActSpeed = Mat_ExeLp_without_min_max(&Emo_Ctrl.SpeedLp, Emo_Ctrl.Speed);
}
