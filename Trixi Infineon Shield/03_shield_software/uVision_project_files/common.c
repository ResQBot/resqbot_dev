
#include "tle_device.h"
#include "common.h"
#include "auto_address.h"

//static bool tmpBool = false;

void sendAnswer(uint16 answer)
{
    SSC2_SendWord(answer);
}

void sendPosAnswer(uint16 answer)
{
    sendAnswer(answer + CONFIRM_OFFSET);
}

static void resetBoard(void)
{
    PMU->GPUDATA00.bit.DATA0 = 0xFF;
    __NVIC_SystemReset();
}

static void ledHandler(uint16 msg, uint8 index, uint8 action)
{
    PORT_ChangePin(index, action);
    sendPosAnswer(msg);
}

bool checkCommand(uint16 msg, uint8 command, uint8 param)
{
		if(command == BOARDCONTROL) {
        PORT_ChangePin(LED_R, PORT_ACTION_CLEAR);  // Red if BOARDCONTROL
    }
    else if(command == LED_ON) {
        PORT_ChangePin(LED_G, PORT_ACTION_CLEAR);  // Green if LED_ON
    }
    else if(command == LED_OFF) {
        PORT_ChangePin(LED_B, PORT_ACTION_CLEAR);  // Blue if LED_OFF
    }
		
    switch(command)
    {		
        case BOARDCONTROL:
            switch(param)
            {
                case AUTOADDRESSING: startAutoaddressing(); break;
                case BOARD_AVAILABLE: 
                    sendPosAnswer(msg);
                    break;
                case RESET: resetBoard(); break;
                default: sendAnswer(msg); break;
            }
            return true;
        case LED_ON: ledHandler(msg, param, PORT_ACTION_CLEAR); return true;
        case LED_OFF: ledHandler(msg, param, PORT_ACTION_SET); return true;
        default: return false;
    }
}

void startAutoaddressing(void)
{
    uint8 address = init_Auto_Addressing();
    setBoardnr(address);
}

void setBoardnr(uint8 boardnr)
{	
    switch(boardnr)
    {
        case 1:
            // Board1 --> Pin 2.3, EXTINT0
            PMU->GPUDATA01.bit.DATA1 = 0x01;
            // Set bits for chip_Selected and chip_Deselected-Interrupt
            SCU->MODPISEL.bit.EXINT0IS = 3;
            // Enable Interrupt at pin 2.3 (User Manual page 143)
            SCU->EXICON0.bit.EXINT0 = 3;
            break;
        case 2:
            // Board2 --> Pin 0.0, EXTINT2
            PMU->GPUDATA01.bit.DATA1 = 0x02;
            // Set bits for chip_Selected and chip_Deselected-Interrupt
            SCU->MODPISEL.bit.EXINT2IS = 3;
            // Enable Interrupt at pin 0.0 (User Manual page 143)
            SCU->EXICON0.bit.EXINT2 = 3;
            break;
        case 3:
            // Board1 --> Pin 0.1, EXTINT0
            PMU->GPUDATA01.bit.DATA1 = 0x03;
            // Set bits for chip_Selected and chip_Deselected-Interrupt
            SCU->MODPISEL.bit.EXINT0IS = 2;
            // Enable Interrupt at pin 0.1 (User Manual page 143)
            SCU->EXICON0.bit.EXINT0 = 3;
            break;
        case 4:
            // Board1 --> Pin 2.4, EXTINT1
            PMU->GPUDATA01.bit.DATA1 = 0x04;
            // Set bits for chip_Selected and chip_Deselected-Interrupt
            SCU->MODPISEL.bit.EXINT1IS = 3;
            // Enable Interrupt at pin 2.4 (User Manual page 143)
            SCU->EXICON0.bit.EXINT1 = 3;
            break;
        default:
            break;
    }
}

void selectSlave(void)
{
    //if(tmpBool) return;
    // Enable SSC2
    PORT->P1_DIR.bit.P2 = 1;
    SSC2->CON.PRG_bit.EN = 1;
}

void deselectSlave(void)
{
    //if(tmpBool) return;
    // Disable SSC2
    PORT->P1_DIR.bit.P2 = 0;
    SSC2->CON.PRG_bit.EN = 0;
    
    //SSC2->TB.reg = 0;
}

void anErrorOccurred(void)
{
    PORT_ChangePin(LED_R, PORT_ACTION_CLEAR);
    PORT_ChangePin(LED_G, PORT_ACTION_SET);
    PORT_ChangePin(LED_B, PORT_ACTION_SET);
}
