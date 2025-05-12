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
#include "Table.h"

/*******************************************************************************
**                      Private Macro Definitions                             **
*******************************************************************************/

  /* 16 patterns for MCMOUT, index = current Hall(BEMF) pattern + direction index 
   * Value = f(current Hall(BEMF) pattern, expected Hall(BEMF) pattern, multi-channel pattern)
   * Note: the order of the sectors in the list is not given from the order of rotation,
   * but by its the binary code defined by the comparators (direction 0 means negative rotation)
   * Sector number binary code
   * 	1				010
   * 	2				011
   * 	3				001
   * 	4				101
   * 	5				100
   * 	6				110
   */

  //CC60 und COUT60 = Phase T : CC61 und COUT61 = Phase S : CC62 und COUT62 = Phase R
  //									CURH	EXPH	MCMP
  //negativ direction
  #define BC_CFG_MCMOUT0 (0x0000u)  //	000		000		000000	--> error: all CC and CCOUT passive
  #define BC_CFG_MCMOUT1 (0x0B31u)	//	001		011		110001	--> COUT60,CC61,COUT61 passive
  #define BC_CFG_MCMOUT2 (0x1607u)  //	010		110		000111	--> COUT61,CC62,COUT62 passive
  #define BC_CFG_MCMOUT3 (0x1A34u)	//	011		010		110100	--> CC60,COUT60,COUT61 passive
  #define BC_CFG_MCMOUT4 (0x251Cu)	//	100		101		011100	--> CC60,COUT60,COUT62 passive
  #define BC_CFG_MCMOUT5 (0x290Du)	//	101		001		001101	--> COUT60,CC62,COUT62 passive
  #define BC_CFG_MCMOUT6 (0x3413u)	//	110		100		010011	--> CC61,COUT61,COUT62 passive
  #define BC_CFG_MCMOUT7 (0x0000u)	//	000		000		000000	--> error all CC and CCOUT passive

  //positive direction
  #define BC_CFG_MCMOUT8  (0x0000u)	//	000		000		000000	--> error: all CC and CCOUT passive
  #define BC_CFG_MCMOUT9  (0x0D0Du)	//	001		101		001101	--> COUT60,CC62,COUT62 passive
  #define BC_CFG_MCMOUT10 (0x1334u)	//	010		011		110100	--> COUT61,CC60,COUT60 passive
  #define BC_CFG_MCMOUT11 (0x1931u)	//	011		001		110001	-->	CC61,COUT61,COUT60 passive
  #define BC_CFG_MCMOUT12 (0x2613u)	//	100		110		010011	-->	COUT62,CC61,COUT61 passive
  #define BC_CFG_MCMOUT13 (0x2C1Cu)	//	101		100		011100	-->	COUT62,CC60,COUT60 passive
  #define BC_CFG_MCMOUT14 (0x3207u)	//	110		010		000111	--> CC62,COUT62,COUT61 passive
  #define BC_CFG_MCMOUT15 (0x0000u) //	000		000		000000	--> error: all CC and CCOUT passive
  /* 16 patterns for MODCTR.T12MODEN, index = current Hall(BEMF) pattern + direction index	*/
  //negative direction
  #define BC_CFG_T12MODEN0 (0x0000u)	//no modulation
  #define BC_CFG_T12MODEN1 (0x0030u)	//CC62 and CCOUT62 are modulated by T12	(CC60 is permanent on, as defined above)
  #define BC_CFG_T12MODEN2 (0x0003u)	//CC60 and CCOUT60 are modulated by T12 (CC61 is permanent on, as defined above)
  #define BC_CFG_T12MODEN3 (0x0030u)	//CC62 and CCOUT62 are modulated by T12	(CC61 is permanent on, as defined above)
  #define BC_CFG_T12MODEN4 (0x000Cu)	//CC61 and CCOUT61 are modulated by T12 (CC62 is permanent on, as defined above)
  #define BC_CFG_T12MODEN5 (0x000Cu)	//CC61 and CCOUT61 are modulated by T12	(CC60 is permanent on, as defined above)
  #define BC_CFG_T12MODEN6 (0x0003u)	//CC60 and CCOUT60 are modulated by T12	(CC62 is permanent on, as defined above)
  #define BC_CFG_T12MODEN7 (0x0000u)	//no modulation

  //positive direction
  #define BC_CFG_T12MODEN8  (0x0000u)	//no modulation
  #define BC_CFG_T12MODEN9  (0x000Cu)	//CC61 and CCOUT61 are modulated by T12 (CC60 is permanent on, as defined above)
  #define BC_CFG_T12MODEN10 (0x0030u)	//CC62 and CCOUT62 are modulated by T12 (CC61 is permanent on, as defined above)
  #define BC_CFG_T12MODEN11 (0x0030u)	//CC62 and CCOUT62 are modulated by T12 (CC60 is permanent on, as defined above)
  #define BC_CFG_T12MODEN12 (0x0003u)	//CC60 and CCOUT60 are modulated by T12 (CC62 is permanent on, as defined above)
  #define BC_CFG_T12MODEN13 (0x000Cu)	//CC61 and CCOUT61 are modulated by T12 (CC62 is permanent on, as defined above)
  #define BC_CFG_T12MODEN14 (0x0003u)	//CC60 and CCOUT60 are modulated by T12 (CC61 is permanent on, as defined above)
  #define BC_CFG_T12MODEN15 (0x0000u)	//no modulation

const uint16 BackEMF_mcp[] =								//constants required for BackEMF-Operation
{
/* multi channel patterns for block commutation */
			BC_CFG_MCMOUT0,  /* Current Hall pattern=0, forward direction (error) */
			BC_CFG_MCMOUT1,  /* Current Hall pattern=1, forward direction */
			BC_CFG_MCMOUT2,  /* Current Hall pattern=2, forward direction */
			BC_CFG_MCMOUT3,  /* Current Hall pattern=3, forward direction */
			BC_CFG_MCMOUT4,  /* Current Hall pattern=4, forward direction */
			BC_CFG_MCMOUT5,  /* Current Hall pattern=5, forward direction */
			BC_CFG_MCMOUT6,  /* Current Hall pattern=6, forward direction */
			BC_CFG_MCMOUT7,  /* Current Hall pattern=7, forward direction (error) */
			BC_CFG_MCMOUT8,  /* Current Hall pattern=0, reverse direction (error) */
			BC_CFG_MCMOUT9,  /* Current Hall pattern=1, reverse direction */
			BC_CFG_MCMOUT10, /* Current Hall pattern=2, reverse direction */
			BC_CFG_MCMOUT11, /* Current Hall pattern=3, reverse direction */
			BC_CFG_MCMOUT12, /* Current Hall pattern=4, reverse direction */
			BC_CFG_MCMOUT13, /* Current Hall pattern=5, reverse direction */
			BC_CFG_MCMOUT14, /* Current Hall pattern=6, reverse direction */
			BC_CFG_MCMOUT15, /* Current Hall pattern=7, reverse direction (error) */
};

const uint8 BackEMF_mod[] =								//constants required for BackEMF-Operation
		//timer 12 modulation enable and multi channel mode enable/disable
{
			(uint8)(0x00 + BC_CFG_T12MODEN0),  /* Current Hall pattern=0, forward direction (error - 0x00 multi channel mode disabled) */
			(uint8)(0x80 + BC_CFG_T12MODEN1),  /* Current Hall pattern=1, forward direction */
			(uint8)(0x80 + BC_CFG_T12MODEN2),  /* Current Hall pattern=2, forward direction */
			(uint8)(0x80 + BC_CFG_T12MODEN3),  /* Current Hall pattern=3, forward direction */
			(uint8)(0x80 + BC_CFG_T12MODEN4),  /* Current Hall pattern=4, forward direction */
			(uint8)(0x80 + BC_CFG_T12MODEN5),  /* Current Hall pattern=5, forward direction */
			(uint8)(0x80 + BC_CFG_T12MODEN6),  /* Current Hall pattern=6, forward direction */
			(uint8)(0x00 + BC_CFG_T12MODEN7),  /* Current Hall pattern=7, forward direction (error - 0x00 multi channel mode disabled) */
			(uint8)(0x00 + BC_CFG_T12MODEN8),  /* Current Hall pattern=0, reverse direction (error - 0x00 multi channel mode disabled) */
			(uint8)(0x80 + BC_CFG_T12MODEN9),  /* Current Hall pattern=1, reverse direction */
			(uint8)(0x80 + BC_CFG_T12MODEN10), /* Current Hall pattern=2, reverse direction */
			(uint8)(0x80 + BC_CFG_T12MODEN11), /* Current Hall pattern=3, reverse direction */
			(uint8)(0x80 + BC_CFG_T12MODEN12), /* Current Hall pattern=4, reverse direction */
			(uint8)(0x80 + BC_CFG_T12MODEN13), /* Current Hall pattern=5, reverse direction */
			(uint8)(0x80 + BC_CFG_T12MODEN14), /* Current Hall pattern=6, reverse direction */
			(uint8)(0x00 + BC_CFG_T12MODEN15)  /* Current Hall pattern=7, reverse direction (error - 0x00 multi channel mode disabled) */
		
};
