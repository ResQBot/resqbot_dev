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
#ifndef MAT_H
#define MAT_H
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
#include <Types.h>
#include "Table.h"

/*******************************************************************************
**                      Global Macro Definitions                              **
*******************************************************************************/
/* Function-like macro to multiply two fixed-point factors to get a fixed-point product with scaling
 * e.g. scale = 1: shift result left by Scale */
#define Mat_FixMulScale(Factor1, Factor2, Scale) ((((sint32)(Factor1)) * ((sint32)(Factor2))) >> ((uint32)((sint32)MAT_FIX_SHIFT - (Scale))))

/* Shift value for fixed-point format */
#define MAT_FIX_SHIFT (15u)

/* Saturation bit for fixed-point format */
#define MAT_FIX_SAT (16u)

/*******************************************************************************
**                      Global Type Definitions                               **
*******************************************************************************/
/** \brief PI status */
typedef struct
{
  sint32 IOut;     /**< \brief I output */
  sint16 Kp;       /**< \brief Proportional parameter */
  sint16 Ki;       /**< \brief Integral parameter */
  sint16 IMin;     /**< \brief Minimum for I output */
  sint16 IMax;     /**< \brief Maximum for I output */
  sint16 PiMin;    /**< \brief Minimum for PI output */
  sint16 PiMax;    /**< \brief Maximum for PI output */
}TMat_Pi;

/** \brief low pass status */
typedef struct
{
  sint16 CoefA;    /**< \brief Coefficient A */
  sint16 CoefB;    /**< \brief Coefficient B */
  sint16 Min;      /**< \brief Minimum */
  sint16 Max;      /**< \brief Maximum */
  sint32 Out;      /**< \brief Low pass output */
}TMat_Lp;

/** \brief low pass status */
typedef struct
{
  sint16 CoefA;    /**< \brief Coefficient A */
  sint16 CoefB;    /**< \brief Coefficient B */
  sint32 Out;      /**< \brief Low pass output */
}TMat_Lp_Simple;


/*******************************************************************************
**                      Global Variable Declarations                          **
*******************************************************************************/

/*******************************************************************************
**                      Global Function Declarations                          **
*******************************************************************************/

/* Inline functions ***********************************************************/
__STATIC_INLINE  sint16 Mat_ExePi(TMat_Pi *pPi, sint16 Error);
__STATIC_INLINE  sint16 Mat_ExeLp(TMat_Lp *pLp, sint16 Input);
__STATIC_INLINE  sint16 Mat_ExeLp_without_min_max(TMat_Lp_Simple *pLp, sint16 Input);
__STATIC_INLINE  sint16 Mat_Ramp(sint16 Input, sint32 Anstieg, sint32 *Output);

/*******************************************************************************
**                      Global Inline Function Definitions                    **
*******************************************************************************/
/** \brief Performs PI control algorithm.
 *
 * \param[inout] pPi Pointer to PI status
 * \param[in] Error Difference between reference and actual value
 *
 * \return PI output
 * \ingroup math_api
 */
__STATIC_INLINE  sint16 Mat_ExePi(TMat_Pi *pPi, sint16 Error)
{
  sint32 IOut;
  sint32 PiOut;
  sint32 Min;
  sint32 Max;
  sint32 Temp;

  /* I output = old output + error * I parameter */
  IOut = pPi->IOut + ((sint32)Error * (sint32)pPi->Ki);
  
  /* Limit I output */
  Min = ((sint32)(pPi->IMin)) << 15u;
  if (IOut < Min)
  {
    IOut = Min;
  }  
  else
  {
    Max = ((sint32)(pPi->IMax)) << 15u;
    if (IOut > Max)
    {
      IOut = Max;
    }  
  }    
  /* Store I output */
  pPi->IOut = IOut;

  /* PI output = upper half of (I output + saturate(error * P parameter) * 64) */
  Temp = __ssat(Error * ((sint32)pPi->Kp), 31u - 6u);
  PiOut = (IOut + (Temp << 6u)) >> 15u;
  
  /* Limit PI output */
  Min = (sint32)(pPi->PiMin);
  if (PiOut < Min)
  {
    PiOut = Min;
  }  
  else
  {
    Max = (sint32)(pPi->PiMax);
    if (PiOut > Max)
    {
      PiOut = Max;
    }  
  }    
  return (sint16)PiOut;

} /* End of Mat_ExePi_Windup */


/** \brief Performs low-pass filter algorithm.
 *
 * \param[inout] pLp Pointer to low-pass filter status
 * \param[in] Input Input in fixed-point format
 *
 * \return Low-pass filter output in fixed-point format
 * \ingroup math_api
 */
__STATIC_INLINE  sint16 Mat_ExeLp(TMat_Lp *pLp, sint16 Input)
{
  sint32 Out;
  sint32 Min;
  sint32 Max;

  /* New output = saturate(old output + coefficient A * input - coefficient B * old output/2^15 */
  Out = pLp->Out;
  Out = __ssat((Out + ((sint32)pLp->CoefA * (sint32)Input)) - ((sint32)pLp->CoefB * (Out >> 15u)), 31u);

  /* Limit new output */
  Min = ((sint32)(pLp->Min)) << 15u;
  if (Out < Min)
  {
    Out = Min;
  }  
  else
  {
    Max = ((sint32)(pLp->Max)) << 15u;
    if (Out > Max)
    {
      Out = Max;
    }  
  }    
  /* Store new output */
  pLp->Out = Out;

  /* return upper part */
  return (sint16)(Out >> 15u);
  
} /* End of Mat_ExeLp */


/** \brief Performs low-pass filter algorithm.
 *
 * \param[inout] pLp Pointer to low-pass filter status
 * \param[in] Input Input in fixed-point format
 *
 * \return Low-pass filter output in fixed-point format without min and max
 * \ingroup math_api
 */
__STATIC_INLINE  sint16 Mat_ExeLp_without_min_max(TMat_Lp_Simple *pLp, sint16 Input)
{
  sint32 Out;

  /* New output = saturate(old output + coefficient A * input - coefficient B * old output/2^15 */
  Out = pLp->Out;
  Out = __ssat((Out + ((sint32)pLp->CoefA * (sint32)Input)) - ((sint32)pLp->CoefB * (Out >> 15u)), 31u);
  /* Store new output */
  pLp->Out = Out;

  /* return upper part */
  return (sint16)(Out >> 15u);
  
} /* End of Mat_ExeLp */


/** \brief Calculates rampe.
 *
 * \param[in] Anstieg Input in fixed-point format
 * \param[out] output32
 * \return output [-32767..+32767]
 *
 * \ingroup math_api
 */
__STATIC_INLINE  sint16 Mat_Ramp(sint16 Input, sint32 Slewrate, sint32 *Output)
{
	sint32 inp;
	sint32 outp;
	outp=*Output;
	inp=Input<<16;
	if (inp>outp){
		outp=outp+Slewrate;
		if (outp>inp){outp=inp;}	
	}
	else {
		outp=outp-Slewrate;
		if (outp<inp){outp=inp;}
	}	
	*Output=outp;
	return(outp>>16);
}	
 /* End of Mat_Ramp */


#endif /* MAT.H */
