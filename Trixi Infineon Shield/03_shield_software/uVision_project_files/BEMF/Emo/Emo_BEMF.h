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
**                      Revision Control History                              **
*******************************************************************************/
/* See Emo_RAM.c */

#ifndef EMO_H
#define EMO_H

/*******************************************************************************
**                      Includes                                              **
*******************************************************************************/
#include <Types.h>
#include "Emo.h"
#include "Mat.h"
#include "Table.h"

/*******************************************************************************
**                      Global Macro Definitions                              **
*******************************************************************************/
/* DC-Link current not in the allowed range */
#define EMO_E_DCLINK_CURRENT_FAILURE (0u)

/* Temperature is above the MaxTemperature */
#define EMO_E_OVER_TEMP_FAILURE      (2u)

/* Over-voltage shutdown */
#define EMO_E_MOTOR_CURRENT_FAILURE  (3u)

/* Motor Start error */
#define EMO_ERROR_VALUE_CU_KI    (0x0001)
#define EMO_ERROR_VALUE_CU_KP    (0x0002)
#define EMO_ERROR_SPEED_POINTS   (0x0020)
#define EMO_ERROR_T_SPEED_LP     (0x0040)
#define EMO_ERROR_STARTTIME      (0x0100)
#define EMO_ERROR_SPEEDSLEWRATE  (0x0200)
#define EMO_ERROR_MINTIME        (0x0400)
#define EMO_ERROR_POLPAIR        (0x0800)

#define EMO_START1_START2_COUNT    (4u)//
#define EMO_START3_START4_COUNT    (2u)
#define EMO_START4_START5_COUNT    (4u)
#define EMO_START5_START6_COUNT     (20u)


/*0 = stays in open-loop operation, 1 = switch into closed-loop operation */
#define EMO_RUN                                   (1u)

/*******************************************************************************
**                      Global Type Definitions                              **
*******************************************************************************/
/** \brief PI configuration */
typedef struct
{
    sint16 Kp;            /**< \brief Proportional parameter */
    sint16 Ki;            /**< \brief Integral parameter */
    sint16 IMin;          /**< \brief Minimum of I output */
    sint16 IMax;          /**< \brief Maximum of I output */
    sint16 PiMin;         /**< \brief Minimum of PI output */
    sint16 PiMax;         /**< \brief Maximum of PI output */
} TEmo_Pi_Cfg;

/** \brief Low pass configuration */
typedef struct
{
    sint16 CoefA;         /**< \brief Coefficient A */
    sint16 CoefB;         /**< \brief Coefficient B */
    sint16 Min;           /**< \brief Minimum */
    sint16 Max;           /**< \brief Maximum */
} TEmo_Lp_Cfg;

/** \brief BEMF status */
typedef struct
{
    uint16 StartAngle;              /**< \brief Angle 0*/
    sint16 StartEndSpeed;      /**< \brief End speed for start 1*/
    uint16 CountStart;          /**< \brief Counter for Start with angle=0 2*/
    uint16 PolePair;          /**< \brief       3                              */
    uint16 TimeSpeedzero;               /**< \brief Time for Speed zero 4*/
    sint16 SpeedtoFrequency;     /**< \brief Time for Speed zero 5*/
    sint32 StartSpeedSlewRate;  /**< \brief Voltage slew rate for start 6*/
    uint16 DcLinkVoltage;             /**< \brief Dc Link Voltage 8*/
    uint16 DcFactor;                        /**< \brief 327670/Dc Link Voltage 9*/
    sint16 StartSpeedSlope;     /**< \brief Start Speed Slope 10*/
    sint16 StartFrequencySlope;  /**< \brief Start Frequency Slope 11*/
    sint32 StartSpeedSlopeMem;  /**< \brief Start Speed Slope Memory 12*/
    uint16 bemf_counter;         /*  14  */
    uint16 bemf_switch;                 /*  15  */
    uint16 sector_no_sp;                  /*  16  */
    uint16 EnableFrZero;
    uint16 bemfc_ctrl_sts_bak;
    uint16 bemfc_ctrl_sts_bak_old;
    uint16 pattern_bak;
    uint16 timestatus;
    uint16 timeoutcounter;
    uint16 difftimeout;
    uint16 speedtestenable;
} TEmo_Bemf;


/** \brief BEMF configuration */
typedef struct
{
    /*Paramater1*/
    float PWM_Frequency;
    uint16 SpeedPi_Kp;        /**< \brief Speedcontroller Kp *64 */
    uint16 SpeedPi_Ki;        /**< \brief Speedcontroller Ki */
    float TimeConstantSpeedFilter;/**< \brief Time constant for Speedfilter*/
    uint16 PolePair;                       /**< \brief Pol Pair counter */
    float TimeSpeedzero;              /**< \brief Time for Speed zero */
    float StartSpeedEnd;       /**< \brief Max Speed for Start */
    float StartSpeedSlewRate;  /**< \brief Start Speed SlewRate */
    float RunSpeedSlewRate;  /**< \brief Run Speed SlewRate */
    uint16 EnableFrZero;              /**< \brief Enable Start width Frequenz=0*/
    uint16 SpeedTestEnable;             /**< \brief Enable Speedtest*/
    float SpeedLevelSwitchOn;    /**< \brief Speedlevel for switch on */
    float StartSpeedPwmMin;     /*Speed pwm start min */
    float StartSpeedPwmMinOffset;     /*Speed pwm start min offset*/
    float SpeedBeginPwmMin;          /*Speed pwm run min */
    float SpeedPwmMin;          /*Speed pwm run min */
} TEmo_Bemfpar_Cfg;


/** \brief Control status */
typedef struct
{
    sint16 RefSpeed;      /**< \brief Reference speed 0*/
    sint16 ActSpeed;      /**< \brief Actual speed 1*/
    sint16 ActSpeeddisplay;  /**< \brief Actuel speed display2*/
    sint16 RefSpeedSlope;    /*3  */
    TMat_Pi SpeedPi;      /**< \brief Speed PI control 4*/
    TMat_Lp_Simple SpeedLp;      /**< \brief Speed low pass 12*/
    sint16 SpeedLevelSwitchOn;  /*16    */
    TMat_Lp_Simple SpeedLpdisplay;      /**< \brief Speed low pass 18 */
    TMat_Lp_Simple DiffTimeLp;      /**< \brief Speed low pass 22*/
    sint16 vout;                                        /* 26  */
    sint16 Period;                             /* 27  */
    sint16 Speed;                                 /* 28  */
    sint16 SpeedPiOut;                      /* 29  */
    sint16 StartVoltage;                    /* 30  */
    uint16 DirectionIndex;            /* 31 */    //rotation direction ( 0 = with the clock, 8 = against the clock)
    uint16 NewTime;                           /* 32 */
    uint16 OldTime;                           /* 33 */
    uint16 DiffTime;                      /* 34 */
    uint32 SpeedConst;                    /* 35 */
    uint16 T3TimeoutCounter;      /* 37 */
    uint16 DiffTimept1;                   /* 38 */
    uint32 SpeedSlewRate;                   /* 39 */
    sint32 RefSpeedSlopeMem;    /**< \brief Start Speed Slope Memory 41 */
    uint16 StartSpeedPwmMin;     /*Speed pwm start min */
    uint16 StartSpeedPwmMinOffset; /*Speed pwm start min */
    uint16 SpeedPwmBeginMin;          /*Speed pwm run min */
    uint16 SpeedPwmMin;          /*Speed pwm run min */
    uint16 StartSpeedPwmFactor;     /*                  */
    uint16 DiffTimeRsp[8];              /*Memory for Difftime  */
    uint16 DiffTimePtr;
    uint16 DiffTimeMw;
    uint16 WronghallCounter;
} TEmo_Ctrl;




/*******************************************************************************
**                      Global Variable Declarations                          **
*******************************************************************************/
extern TEmo_Status Emo_Status;
extern  TEmo_Ctrl Emo_Ctrl;
extern TEmo_Bemfpar_Cfg Emo_Bemfpar_Cfg;

/* datasets */
typedef struct 
{
	TEmo_Bemfpar_Cfg emocfg_BEMF;
	// additional parameters that can be saved
	uint16 spike_filt;
	uint16 blank_filt;
}Tdatastruct_BEMF;

/* from Infineon --> right after code */
extern const Tdatastruct_BEMF dataset0 __attribute__((at(0x11008000)));
extern const Tdatastruct_BEMF dataset1 __attribute__((at(0x11008080)));
extern const Tdatastruct_BEMF dataset2 __attribute__((at(0x11008100)));
extern const Tdatastruct_BEMF dataset3 __attribute__((at(0x11008180)));
/* custom --> last sector of flash */
extern const Tdatastruct_BEMF customdataset0 __attribute__((at(0x1101C000)));
extern const Tdatastruct_BEMF customdataset1 __attribute__((at(0x1101C080)));
extern const Tdatastruct_BEMF customdataset2 __attribute__((at(0x1101C100)));
extern const Tdatastruct_BEMF customdataset3 __attribute__((at(0x1101C180)));

extern  TEmo_Ctrl Emo_Ctrl;

extern TEmo_Bemfpar_Cfg Emo_Bemfpar_Cfg;
extern TEmo_Bemf Emo_Bemf;
extern uint16 PolePair;

extern sint16 TestVar[4];

/*******************************************************************************
**                      Global Function Declarations                          **
*******************************************************************************/

extern uint16 speeduserreferenz;
extern sint16 iqusertorquereferenz;
extern uint16 controlmode;

extern void BackEMF_Handle_Correct(void);
extern void BackEMF_Handle_Wrong(void);
extern void Pwm_Handle(void);

extern void Emo_setspeedreferenz(sint16 speedreferenz);


/* Inline functions ***********************************************************/

extern sint16 abs(sint16 inp);

/*******************************************************************************
**       *               Global Inline Function Definitions                    **
*******************************************************************************/

#endif /* EMO_H */
