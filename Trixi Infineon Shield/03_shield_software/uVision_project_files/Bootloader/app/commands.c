
#include "commands.h"
#include <wdt1.h>
#include "tle_device.h"
#include "bootrom.h"
#include "SSC.h"
#include "..\..\auto_address.h"

//--------------------0x01: changeMode--------------------
void changeMode(uint8 requestedMode)
{
	// Konstanten fuer Bootloader
	int (*const USER_BEMF_ENTRY)  (void) = (int (*)(void)) (*(( int * ) (BEMF_IMAGE_START + 4)));
	int (*const USER_HALL_ENTRY)  (void) = (int (*)(void)) (*(( int * ) (HALL_IMAGE_START + 4)));
	int (*const USER_FOC_ENTRY)   (void) = (int (*)(void)) (*(( int * ) (FOC_IMAGE_START + 4)));
	int (*const USER_BLINK_ENTRY) (void) = (int (*)(void)) (*(( int * ) (BLINK_IMAGE_START + 4)));
	
	// wait until watchdog has just been reset --> enough time to jump
	while(WDT1_Service() != 1){};
	
	// im Bootloader: Sprung in andere Teil-Applikation
	switch(requestedMode)
	{
		case 0x01: 
			SCU->EXICON0.reg = (uint8)0x00; // disable external interrupts
			SSC2->CON.PRG_bit.EN = 0; // Disable SSC2
			__disable_irq();
			USER_BEMF_ENTRY();
			break;
		case 0x02: 
			SCU->EXICON0.reg = (uint8)0x00; // disable external interrupts
			SSC2->CON.PRG_bit.EN = 0; // Disable SSC2
			__disable_irq();
			USER_HALL_ENTRY();
			break;
		case 0x03: 
			SCU->EXICON0.reg = (uint8)0x00; // disable external interrupts
			SSC2->CON.PRG_bit.EN = 0; // Disable SSC2
			__disable_irq();
			USER_FOC_ENTRY();
			break;
		case 0x04: 
			SCU->EXICON0.reg = (uint8)0x00; // disable external interrupts
			SSC2->CON.PRG_bit.EN = 0; // Disable SSC2
			__disable_irq();
			USER_BLINK_ENTRY();
			break;
		default:
			break;
	}
}
