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
 * V1.0.0: 2016-07-20, ISLE:   Initial version
 */

/******************************************************************************/
/** BLDC: Motor Drive with sensorless block commutation (BEMF)               **/
/** configured to with the Trinamic QBL 4208-61-04-013 motor                 **/
/******************************************************************************/
/** runs on MON button press on a speed of 2000rpm                           **/
/******************************************************************************/

/*******************************************************************************
**                      Includes                                              **
*******************************************************************************/
#include "Main.h"
#include "Emo_BEMF.h"
#include "globvars_BEMF.h"
#include "commands.h"
#include "..\..\common.h"
#include "..\..\commonMC.h"


/*******************************************************************************
**                      Private Macro Definitions                             **
*******************************************************************************/
#define CURRENTMODE 1

/*******************************************************************************
**                      Private Function Declarations                         **
*******************************************************************************/
static void Main_lStartMotor(void);
static void Main_lStopMotor(void);

/*******************************************************************************
**                      Globale Variable Definitions                          **
*******************************************************************************/

/*******************************************************************************
**                      Private Variable Definitions                          **
*******************************************************************************/
static uint8 bMotorRun=0;

// interrupt --> main
static uint16 word1 = 0;
static uint16 word_buffer = 0;
static uint8 word1byte1 = 0;
static uint8 word1byte2 = 0;
static uint16 word2 = 0;
static uint16 word3 = 0;
static sint16 motorspeed = 0;

// main --> interrupt
static uint8 data_prepared = 0;
static uint8 success = 0;
static uint8 success_save=0;
static uint8 calcCRC;
static uint8 recvCRC;
 

// main <--> interrupt
uint8 cmdstate = UNINITIALIZED;

/*******************************************************************************
**                      Private Constant Definitions                          **
*******************************************************************************/

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
  /*****************************************************************************
  ** initialization of the hardware modules based on the configuration done   **
  ** by using the IFXConfigWizard                                             **
  *****************************************************************************/
    uint8 boardnr = 0;
    uint8 *uint8ptr;
    uint16 act_speed=0; 

    /* Initialize device drivers, Note: Watchdog is already initialized in Bootloader */
    TLE_Init();
    
    // set Chip_Selected- and Chip_Deselected-interrupts according to boardnr in GPUDATA01
    boardnr = PMU->GPUDATA01.bit.DATA1;
    setBoardnr(boardnr);
    
    // BEMF color
    PORT_ChangePin(LED_R, PORT_ACTION_SET);
    PORT_ChangePin(LED_G, PORT_ACTION_CLEAR);
    PORT_ChangePin(LED_B, PORT_ACTION_CLEAR);
    sendPosAnswer(0x0100 + CURRENTMODE);
    
    bemfEndSpeedPwmMin = Emo_Bemfpar_Cfg.StartSpeedPwmMin + Emo_Bemfpar_Cfg.StartSpeedPwmMinOffset;
    rxtxbuffer.datastruct_BEMF.emocfg_BEMF = Emo_Bemfpar_Cfg;
    Emo_SetRefSpeed(3000);
    Emo_Init();
    
    cmdstate = READYFORSPIDATA;

    while (1)
    {
        /* Service watch-dog */
        WDT1_Service();

        /* wait for new spi data */
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
                    
                case LOADDATASET: 
                     if(bMotorRun == 1u)
                        {
                            Main_lStopMotor();
                            bMotorRun = 0u;
                        }
                    word_buffer = word1;
                    loadDataset(word1byte2);
                    success=1;		
                    cmdstate = READYFORSPIDATA;
                    break;
                        
                case SENDDATASET:
                    // just prepare data here, data is sent in interrupt
                    if(bMotorRun == 1u)
                        {
                            Main_lStopMotor();
                            bMotorRun = 0u;
                        }
                    rxtxbuffer.datastruct_BEMF.emocfg_BEMF = Emo_Bemfpar_Cfg;
                    rxtxbuffer.datastruct_BEMF.spike_filt = (uint16)(MF->TRIM_BEMFx.reg & 0x00000300) >> 8;
                    rxtxbuffer.datastruct_BEMF.blank_filt = (uint16)(MF->TRIM_BEMFx.reg & 0x00000007);
                    if(rxtxbuffer.datastruct_BEMF.blank_filt > 4)
                        rxtxbuffer.datastruct_BEMF.blank_filt = 4;
                    
                    uint8ptr = (uint8*)&rxtxbuffer;
                    calcCRC =  CRC8(uint8ptr, NROFBYTES);
                    data_prepared = 1;
                    cmdstate = READYFORSPIDATA;
                    break;
                    
                case RECEIVEDATASET:
                    if(bMotorRun == 1u)
                        {
                            Main_lStopMotor();
                            bMotorRun = 0u;
                        }
                    word_buffer = word1;				
                    uint8ptr = (uint8*)&rxtxbuffer;
                    calcCRC =  CRC8(uint8ptr, NROFBYTES);
                    
                    if(recvCRC == calcCRC)
                    {
                        if(bMotorRun == 1u)
                        {
                            Main_lStopMotor();
                            bMotorRun = 0u;
                        }
                        Emo_Bemfpar_Cfg = rxtxbuffer.datastruct_BEMF.emocfg_BEMF;
                        MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFCFF) | (rxtxbuffer.datastruct_BEMF.spike_filt << 8);
                        MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFFF8) | rxtxbuffer.datastruct_BEMF.blank_filt;
                        Emo_Update();   
                        success=1;			
                    }
                    else
                    {
                        success=0;	
                    }
                    cmdstate = READYFORSPIDATA;
                    break;
                        
                case CHANGEPARAMETER:
                    word_buffer = word1;
                    if(bMotorRun == 1u)
                    {
                        Main_lStopMotor();
                        bMotorRun = 0u;
                    }
                    changeSingleParameter(word1byte2, word2, word3);
                    success=1;	
                    cmdstate = READYFORSPIDATA;
                    break;
                        
                case SAVEDATASET:
                    word_buffer = word1;					
                    success_save = saveCurrentDataset(word1byte2);
                    // saving dataset needs more time than other commands --> SSC not enabled anymore
                    // For writing answer to transmit-register SSC needs to be enabled, disable after writing answer to register
                    if(success_save == 0) // saving was successful
                    {
                        success=1;
                    } else // an error occured (1: position was not valid; 2: the checksums don't match; 3: the write protection was not off)
                    {
                        success=0;
                    }
                    cmdstate = READYFORSPIDATA;
                    break;
                    
                case SETMOTORSPEED:
                    word_buffer = word1;
                    Emo_SetRefSpeed(motorspeed);	
                    success=1;	
                    cmdstate = READYFORSPIDATA;
                    break;

                /* Update: GETMOTORSPEED is used to get the actual speed of rotation  */ 
                case GETMOTORSPEED: 
                    act_speed=Emo_Ctrl.ActSpeeddisplay;
                    SSC2_SendWord(act_speed);
                    cmdstate = READYFORSPIDATA;
                    break;
                
                case MOTORCONTROL:
                    word_buffer = word1;
                    if((word1byte2 == START_MOTOR) && (bMotorRun == 0u)) 
                    {	
                        // Initialize and start motor
                        Main_lStartMotor();
                        bMotorRun = 1u;
                    }
                    if (word1byte2 == STOP_MOTOR)	
                    {
                        // Stop  motor
                        Main_lStopMotor();
                        bMotorRun = 0u;
                    }
                    success=1;	
                    cmdstate = READYFORSPIDATA;
                    break;
                    
                    case CHECKSUCCESS:		
                        if(success == 1) // saving was successful
                        {
                            sendPosAnswer(word_buffer);
                            success=0;
                        }
                        else
                        {
                            sendAnswer(word_buffer);
                        }
                        cmdstate = READYFORSPIDATA;
                        break;					
                    
                case CHECK_ERROR:
                    checkErrorsMC(&(Emo_Status.MotorStartError));
                    cmdstate = READYFORSPIDATA;
                    break;
                
                default:
                    /* wrong command */
                    cmdstate = READYFORSPIDATA;
                    break;
            }
        }
  }
} /* End of main() */


void Main_HandleSysTick(void)
{

} /* End of Main_HandleSysTick */


/*******************************************************************************
**                      Private Function Definitions                          **
*******************************************************************************/
static void Main_lStartMotor(void)
{
    Emo_StartMotor();
}



static void Main_lStopMotor(void)
{
    Emo_StopMotor();
}
/* End of Main_lStopMotor */


// ----------------------------------------interrupts----------------------------------------
void Data_Received()
{
    uint16 receivedword = SSC2_ReadWord();
    //internal interrupt state
    static uint8 waitingFor = NEWCOMMAND; 
    static uint8 sendcounter = 0, receivecounter = 0, messagesToReceive = 0;
    
    if((cmdstate != READYFORSPIDATA) && (receivedword != 0x0902)) //BOARDCONTROL, RESET
        return;

    switch (waitingFor)
    {
        case NEWCOMMAND:  // ------------------------------ NEWCOMMAND ------------------------------
            word1 = receivedword;
            word1byte1 = 0xFF & (receivedword >> 8);
            word1byte2 = 0xFF & receivedword;
            
            switch (word1byte1)
            {
                /* catch "special" cases that need another Message */
                case 0x03:
                    /* send data to master when receiving next message, now send number of 16bit-messages to send */
                    SSC2_SendWord(NROFMESSAGES);
                    waitingFor = SENDDATA;
                    cmdstate = NEWSPIDATATOPROCESS;
                    break;
                case 0x04:
                    waitingFor = RECEIVEDATA;
                    messagesToReceive = word1byte2;
                    break;
                case 0x05:
                    if (isValueInArray(word1byte2, indices_16bit, indices_16bit_size) == 1) /* 16Bit data --> wait for one more message */
                    {
                        waitingFor = DATA_1_OF_1;
                    }
                    else /* 32Bit data --> wait for two more messages */
                    {
                        waitingFor = DATA_1_OF_2;
                    }
                    break;
                case 0x07: /* motorspeed: wait for one more message (=motorspeed) */
                    waitingFor = MOTORSPEED;
                    break;
                case GETMOTORSPEED:  // Update: for get motor speed	
                    waitingFor = NEWCOMMAND;
                    cmdstate = NEWSPIDATATOPROCESS;
                    break;				
                default: /* just one message for this command --> data complete */
                    cmdstate = NEWSPIDATATOPROCESS;
                    break;
            }
            break;
        
        case DATA_1_OF_1:  
            word2 = receivedword;
            waitingFor = NEWCOMMAND;
            cmdstate = NEWSPIDATATOPROCESS;
            break;
        
        case DATA_1_OF_2: 
            word2 = receivedword;
            waitingFor = DATA_2_OF_2;
            break;
        
        case DATA_2_OF_2:  
            word3 = receivedword;
            waitingFor = NEWCOMMAND;
            cmdstate = NEWSPIDATATOPROCESS;
            break;

        case MOTORSPEED:  
            // receivedword (unsigned int) contains an signed value! --> first bit = 1 --> negative
            if(receivedword > 32767)
                motorspeed = 0x8000 + (receivedword & 0x7FFF);
            else
                motorspeed = receivedword;
            
            waitingFor = NEWCOMMAND;
            cmdstate = NEWSPIDATATOPROCESS;
            break;
        
        case SENDDATA:  // ------------------------------ SENDDATA ------------------------------
            // send one 16bit-message at each interrupt (only after data is prepared in main loop)
            if (data_prepared == 1)
            {
                if(sendcounter < NROFMESSAGES)
                {
                    SSC2_SendWord(rxtxbuffer.dataarray_BEMF[sendcounter]);
                    sendcounter++;
                }
                else  // send CRC
                {
                    SSC2_SendWord(calcCRC);
                    sendcounter = 0;
                    data_prepared = 0;
                    waitingFor = NEWCOMMAND;
                    cmdstate = READYFORSPIDATA;
                }
            }
            break;
            
        case RECEIVEDATA:  // ------------------------------ RECEIVEDATA ------------------------------
            if(receivecounter < messagesToReceive)
            {
                /* collect data here and write them to emo config variable when complete (in main, not in interrupt) */
                rxtxbuffer.dataarray_BEMF[receivecounter] = receivedword;
                receivecounter++;
            }
            else
            {
                recvCRC = (uint8_t)receivedword;
                receivecounter++;				
                
                // all messages received
                receivecounter = 0;
                messagesToReceive = 0;
                waitingFor = NEWCOMMAND;
                cmdstate = NEWSPIDATATOPROCESS;
            }				
            break;
            
        default:  // ------------------------------ default ------------------------------
            break;
    } /* end of switch (waitingFor) */
}
