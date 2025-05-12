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
/* See mon.c */

#ifndef MON_H
#define MON_H

/*******************************************************************************
**                      Includes                                              **
*******************************************************************************/
#include <tle_device.h>
#include <Types.h>
#include "mon_defines.h"

/******************************************************************************
**                      Global Macro Definitions                             **
*******************************************************************************/
/* MON Status Flag mask */
#define MON_STATUS_LOW (0U)
#define MON_STATUS_HIGH (1U)

/* MON Interrupt Clear Macros */
#define MON_Rising_Edge_Int_Clr()    (SCU->IRCON0CLR.reg = (uint8)1u << 6u)
#define MON_Falling_Edge_Int_Clr()   (SCU->IRCON0CLR.reg = (uint8)1u << 7u)

/* MON Interrupt Enable/Disable Macros */
#define MON_Rising_Edge_Int_En()     (SCU->EXICON0.reg |= (uint8)(1u << 6u))
#define MON_Rising_Edge_Int_Dis()    (SCU->EXICON0.reg &= (uint8)~(1u << 6u))
#define MON_Falling_Edge_Int_En()    (SCU->EXICON0.reg |= (uint8)(1u << 7u))
#define MON_Falling_Edge_Int_Dis()   (SCU->EXICON0.reg &= (uint8)~(1u << 7u))

/* MON Control and Status Macros */
#define MON_Sts()                   ((SCU->MODIEN3.reg >> 5u) & 1u)
#define MON_PullUp_En()             (MON->CNF.reg |= (uint32)  (1u << 5u))
#define MON_PullUp_Dis()            (MON->CNF.reg &= (uint32) ~(1u << 5u))
#define MON_PullDown_En()           (MON->CNF.reg |= (uint32)  (1u << 4u))
#define MON_PullDown_Dis()          (MON->CNF.reg &= (uint32) ~(1u << 4u))
#define MON_CycSense_En()           (MON->CNF.reg |= (uint32)  (1u << 3u))
#define MON_CycSense_Dis()          (MON->CNF.reg &= (uint32) ~(1u << 3u))
#define MON_WakeOnRise_En()         (MON->CNF.reg |= (uint32)  (1u << 2u))
#define MON_WakeOnRise_Dis()        (MON->CNF.reg &= (uint32) ~(1u << 2u))
#define MON_WakeOnFall_En()         (MON->CNF.reg |= (uint32)  (1u << 1u))
#define MON_WakeOnFall_Dis()        (MON->CNF.reg &= (uint32) ~(1u << 1u))
#define MON_En()                    (MON->CNF.reg |= (uint32)  (1u << 0u))
#define MON_Dis()                   (MON->CNF.reg &= (uint32) ~(1u << 0u))

/*******************************************************************************
**                      Global Function Declarations                          **
*******************************************************************************/
__STATIC_INLINE uint8 MON_Get_Status(void);
void MON_Init(void);
uint8 MON_Debounce(uint8 MonActiveState);

/*******************************************************************************
**                      Global INLINE Function Definitions                    **
*******************************************************************************/
/** \brief Reads out the MON Status
 *
 * \param[in] none
 * \param[in] MON status flag
 * \return None
 *
 * \ingroup int_api
 */
__STATIC_INLINE uint8 MON_Get_Status(void)
{
  return ((uint8) SCU->MODIEN3.bit.MONSTS);
}

#endif
