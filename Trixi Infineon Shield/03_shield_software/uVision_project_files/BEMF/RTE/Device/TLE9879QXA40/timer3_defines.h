/*sha256=FEC372CBF06ED53C6E8DF51452322D9C95649E82656AAFD4A93BEE832CC71B48*/
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

/*------------------------------------------------------------------------------
IFXConfigWizard output file
created on:Do Aug 17 10:38:58 2017
------------------------------------------------------------------------------*/

#ifndef _TIMER3_DEFINES_H
#define _TIMER3_DEFINES_H

#ifndef IFXConfigWizard_Version
  #define IFXConfigWizard_Version 1.8.6
#endif

/* XML Version 1.3.0 */
#ifndef TIMER3_XML_VERSION
  #define TIMER3_XML_VERSION 10300
#endif

#ifndef TIMER3_CLK
  #define TIMER3_CLK 20
#endif

/*TIMER3_CMP: 65535*/
#ifndef TIMER3_CMP
  #define TIMER3_CMP (0xFFFFu)
#endif

/*TIMER3_CTRL: 0|(0<<2)|(0<<9)|(0<<8)*/
#ifndef TIMER3_CTRL
  #define TIMER3_CTRL (0x0u)
#endif

#ifndef TIMER3_MAX_PERIOD
  #define TIMER3_MAX_PERIOD 3276.75
#endif

/*TIMER3_MODE0_VALUE: 8191*/
#ifndef TIMER3_MODE0_VALUE
  #define TIMER3_MODE0_VALUE (0x1FFFu)
#endif

#ifndef TIMER3_MODE0_VALUE_SEL
  #define TIMER3_MODE0_VALUE_SEL 0
#endif

#ifndef TIMER3_MODE0_VALUE_TICK
  #define TIMER3_MODE0_VALUE_TICK 1
#endif

#ifndef TIMER3_MODE0_VALUE_TIME
  #define TIMER3_MODE0_VALUE_TIME 1
#endif

/*TIMER3_MODE1_CMP: 65535*/
#ifndef TIMER3_MODE1_CMP
  #define TIMER3_MODE1_CMP (0xFFFFu)
#endif

#ifndef TIMER3_MODE1_CMP_SEL
  #define TIMER3_MODE1_CMP_SEL 0
#endif

#ifndef TIMER3_MODE1_CMP_TICK
  #define TIMER3_MODE1_CMP_TICK 0
#endif

#ifndef TIMER3_MODE1_CMP_TIME
  #define TIMER3_MODE1_CMP_TIME 0
#endif

/*TIMER3_MODE1_VALUE: 65535*/
#ifndef TIMER3_MODE1_VALUE
  #define TIMER3_MODE1_VALUE (0xFFFFu)
#endif

#ifndef TIMER3_MODE1_VALUE_SEL
  #define TIMER3_MODE1_VALUE_SEL 0
#endif

#ifndef TIMER3_MODE1_VALUE_TICK
  #define TIMER3_MODE1_VALUE_TICK 1
#endif

#ifndef TIMER3_MODE1_VALUE_TIME
  #define TIMER3_MODE1_VALUE_TIME 1
#endif

/*TIMER3_MODE2_CMP: 235*/
#ifndef TIMER3_MODE2_CMP
  #define TIMER3_MODE2_CMP (0xEBu)
#endif

#ifndef TIMER3_MODE2_CMP_SEL
  #define TIMER3_MODE2_CMP_SEL 0
#endif

/*TIMER3_MODE2_CMP_TICK: 0*/
#ifndef TIMER3_MODE2_CMP_TICK
  #define TIMER3_MODE2_CMP_TICK (0x0u)
#endif

#ifndef TIMER3_MODE2_CMP_TIME
  #define TIMER3_MODE2_CMP_TIME 0
#endif

/*TIMER3_MODE2_RL_VALUE: 235*/
#ifndef TIMER3_MODE2_RL_VALUE
  #define TIMER3_MODE2_RL_VALUE (0xEBu)
#endif

#ifndef TIMER3_MODE2_RL_VALUE_SEL
  #define TIMER3_MODE2_RL_VALUE_SEL 0
#endif

/*TIMER3_MODE2_RL_VALUE_TICK: 1*/
#ifndef TIMER3_MODE2_RL_VALUE_TICK
  #define TIMER3_MODE2_RL_VALUE_TICK (0x1u)
#endif

#ifndef TIMER3_MODE2_RL_VALUE_TIME
  #define TIMER3_MODE2_RL_VALUE_TIME 1
#endif

/*TIMER3_MODE2_VALUE: 235*/
#ifndef TIMER3_MODE2_VALUE
  #define TIMER3_MODE2_VALUE (0xEBu)
#endif

#ifndef TIMER3_MODE2_VALUE_SEL
  #define TIMER3_MODE2_VALUE_SEL 0
#endif

/*TIMER3_MODE2_VALUE_TICK: 1*/
#ifndef TIMER3_MODE2_VALUE_TICK
  #define TIMER3_MODE2_VALUE_TICK (0x1u)
#endif

#ifndef TIMER3_MODE2_VALUE_TIME
  #define TIMER3_MODE2_VALUE_TIME 1
#endif

/*TIMER3_MODE3_CMP: 1*/
#ifndef TIMER3_MODE3_CMP
  #define TIMER3_MODE3_CMP (0x1u)
#endif

/*TIMER3_MODE_CONF: (0<<6)|(0<<7)|1*/
#ifndef TIMER3_MODE_CONF
  #define TIMER3_MODE_CONF (0x1u)
#endif

/*TIMER3_T3: 65535*/
#ifndef TIMER3_T3
  #define TIMER3_T3 (0xFFFFu)
#endif

/*TIMER3_TRIGG_CTRL: 3|(0<<6)|(1<<4)*/
#ifndef TIMER3_TRIGG_CTRL
  #define TIMER3_TRIGG_CTRL (0x13u)
#endif


#endif
