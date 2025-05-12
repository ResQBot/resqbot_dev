
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
#include <tle_device.h>
/*
#include <wdt1.h>
#include <scu.h>
#include <port.h>
#include <int.h>
#include <ssc.h>
*/
#include "main.h"
#include <adc1.h>
#include "commands.h"
#include "..\..\common.h"

/*******************************************************************************
**                      Private Macro Definitions                             **
*******************************************************************************/
#define CURRENTMODE 0

/*******************************************************************************
**                      Private Variable Definitions                          **
*******************************************************************************/
// interrupt --> main
static uint16 word1 = 0;
static uint8 word1byte1 = 0, word1byte2 = 0;
static uint16 errorCode = 0;

// main <--> interrupt
uint8 cmdstate = UNINITIALIZED;


/*******************************************************************************
**                      Global Function Definitions                           **
*******************************************************************************/
/** \brief Executes main code.
 *
 * \param None
 * \return None
 *
 */
int main(void)
{
	//uint8 boardnr = 0;
	uint8 requestedMode = 0;
	
	// Initialize all device functionalities
	TLE_Init();
	
	// was a mode change requested? yes --> go to requested mode now
	requestedMode = PMU->GPUDATA00.bit.DATA0;
	
	//setBoardnr(0x01);
	//requestedMode = 0x02;
	
	PMU->GPUDATA00.bit.DATA0 = 0x00;
	if((requestedMode == 0x01) || (requestedMode == 0x02) || (requestedMode == 0x03)) // BEMF, HALL or FOC
	{
		changeMode(requestedMode);
	}
	
	sendPosAnswer(0x0100 + CURRENTMODE);
	
	// Bootloader --> LED 0.1, 0.2 and 0.3 on
	PORT_ChangePin(LED_R, PORT_ACTION_CLEAR);
	PORT_ChangePin(LED_G, PORT_ACTION_CLEAR);
	PORT_ChangePin(LED_B, PORT_ACTION_CLEAR);
	
	// auto-address the boards
	startAutoaddressing();
	
	cmdstate = READYFORSPIDATA;
	
	while (1)
	{
		// Service watch-dog
		WDT1_Service();
	
		// wait for new spi data
		if(cmdstate == NEWSPIDATATOPROCESS)
		{
			cmdstate = PROCESSINGSPIDATA;
			
			if(checkCommand(word1, word1byte1, word1byte2))
			{
				cmdstate = READYFORSPIDATA;
				continue;
			}
			
			switch (word1byte1)
			{
				case MODECONTROL:
					if(word1byte2 == GETMYMODE) sendPosAnswer(word1 + CURRENTMODE);
					else if(CURRENTMODE != word1byte2) changeMode(word1byte2);
					else sendAnswer(word1);
					cmdstate = READYFORSPIDATA;
					break;
				case CHECK_ERROR:
					cmdstate = READYFORSPIDATA;
					if(errorCode != ERR_NONE)
					{
						sendAnswer(word1 + errorCode);
						errorCode = ERR_NONE;
						break;
					}
					sendPosAnswer(word1);
					break;
				default: // wrong command
					cmdstate = READYFORSPIDATA;
					break;
			}
		}
  }
} // End of main()

// ----------------------------------------interrupts----------------------------------------
void Data_Received()
{
	uint16 receivedword = SSC2_ReadWord();
	
	if((cmdstate != READYFORSPIDATA) && (receivedword != 0x0902)) //BOARDCONTROL, RESET
		return;
	
	word1 = receivedword;
	word1byte1 = 0xFF & (receivedword >> 8);
	word1byte2 = 0xFF & receivedword;
	
	cmdstate = NEWSPIDATATOPROCESS;
}
