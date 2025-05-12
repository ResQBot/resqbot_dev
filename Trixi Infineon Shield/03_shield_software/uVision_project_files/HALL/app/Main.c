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

/******************************************************************************/
/** BLDC: Motor Drive with block commutation and Hall sensor                 **/
/******************************************************************************/
/** use the Poti to start/stop and speedup motor                             **/
/** Motor connection, QBL4208:                                               **/
/** Phase 1             : black                                              **/
/** Phase 2             : yellow                                             **/
/** Phase 3             : red                                                **/
/** Hall A - P2.0       : blue			(GND level)                           	 **/
/** Hall B - P1.4       : green     (VDDEXT level)                           **/
/** Hall C - P2.2       : white     (NOTHING level)                          **/
/** Hall Supply - VDDEXT: red                                                **/
/** Hall Gnd - GND      : black                                              **/
/******************************************************************************/

/*******************************************************************************
**                      Includes                                              **
*******************************************************************************/
#include "Main.h"
#include "Emo.h"
#include "globvars_HALL.h"
#include "commands.h"
#include "..\..\common.h"
#include "..\..\commonMC.h"

/*******************************************************************************
**                      Private Macro Definitions                             **
*******************************************************************************/
#define CURRENTMODE 2

/* states for interrupt */
#define NEWCOMMAND 0
#define DATA_1_OF_1 1
#define DATA_1_OF_2 2
#define DATA_2_OF_2 3
#define MOTORSPEED 4
#define SENDDATA 5
#define RECEIVEDATA 6

/* states for communication main <--> interrupt */
#define UNINITIALIZED				0
#define READYFORSPIDATA 		1
#define NEWSPIDATATOPROCESS	2
#define PROCESSINGSPIDATA 	3

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

uint16 errorState = ERR_NONE;

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
    sint16 act_speed=0; // Update: for get speed
	
	 /**
	  * \brief Storage variable for readHallPattern
	  * \author Beatrix Dietl
	  */
		uint32 hallPattern = 0;
    //uint32 tmpCounter = 0;

    /* Initialize device drivers, Note: Watchdog is already initialized in Bootloader */
    TLE_Init();
    
    // set Chip_Selected- and Chip_Deselected-interrupts according to boardnr in GPUDATA01
    boardnr = PMU->GPUDATA01.bit.DATA1;
    setBoardnr(boardnr);
    
    // HALL --> blue
    PORT_ChangePin(LED_R, PORT_ACTION_SET);
    PORT_ChangePin(LED_G, PORT_ACTION_SET);
    PORT_ChangePin(LED_B, PORT_ACTION_CLEAR);
    
    // Initialization complete -> report back to Arduino
    sendPosAnswer(0x0100 + CURRENTMODE);
    
    rxtxbuffer.datastruct_HALL = Emo_Hallpar_Cfg;
    Emo_SetRefSpeed(1000);
    errorState = Emo_Init();
    
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
                    word_buffer = word1;
                    if(bMotorRun == 1u)
                        {
                            Main_lStopMotor();
                            bMotorRun = 0u;
                        }
                    loadDataset(word1byte2);
                    success=1;
                    cmdstate = READYFORSPIDATA;
                    break;
                        
                case SENDDATASET:
                    if(bMotorRun == 1u)
                        {
                            Main_lStopMotor();
                            bMotorRun = 0u;
                        }
                    // just prepare data here, data is sent in interrupt
                    rxtxbuffer.datastruct_HALL = Emo_Hallpar_Cfg;
                    uint8ptr = (uint8*)&rxtxbuffer;
                    calcCRC =  CRC8(uint8ptr, NROFBYTES);
                    data_prepared = 1;
                    cmdstate = READYFORSPIDATA;
                    break;
                    
                case RECEIVEDATASET: 
                    word_buffer = word1;
                    if(bMotorRun == 1u)
                        {
                            Main_lStopMotor();
                            bMotorRun = 0u;
                        }
                    uint8ptr = (uint8*)&rxtxbuffer;
                    calcCRC =  CRC8(uint8ptr, NROFBYTES);
                    if(recvCRC == calcCRC)
                    {
                        Emo_Hallpar_Cfg = rxtxbuffer.datastruct_HALL;
                        Emo_update();
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
                    success = saveCurrentDataset(word1byte2);
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
                
                    case GETMOTORSPEED:
                    if(motorspeed<0)
                        act_speed=-1*(int16_t)EmoCcu_HallStatus.Speed;
                    else
                        act_speed=(int16_t)EmoCcu_HallStatus.Speed;
                    SSC2_SendWord(act_speed);
                    cmdstate = READYFORSPIDATA;
                    break;
                    
                case MOTORCONTROL:
                    word_buffer = word1;
                    if((word1byte2 == START_MOTOR) && (bMotorRun == 0u)) 
                    {	
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
								
							 /**
								* \brief case for READHALLPATTERN command
								* \author Beatrix Dietl
								*/
								case READHALLPATTERN:
										hallPattern = readHallPattern();
										SSC2_SendWord(hallPattern);
										cmdstate = READYFORSPIDATA;
										break;
                    
                    case CHECKSUCCESS:
                        if(success == 1) 
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
                    checkErrorsMC(&errorState);
                    cmdstate = READYFORSPIDATA;
                    
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
  /* Callback function executed every ms for speed control */
  Emo_CtrlSpeed();
} /* End of Main_HandleSysTick */


/*******************************************************************************
**                      Private Function Definitions                          **
*******************************************************************************/
static void Main_lStartMotor(void)
{
    errorState = Emo_StartMotor();
} /* End of Main_lStartMotor */


static void Main_lStopMotor(void)
{
  errorState = Emo_StopMotor();
} /* End of Main_lStopMotor */


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
							 /**
							  * \brief case for READHALLPATTERN command
							  * \author Beatrix Dietl
							  */
								case READHALLPATTERN:
										waitingFor = NEWCOMMAND;
										cmdstate = NEWSPIDATATOPROCESS;
										break;
                default: /* just one message for this command --> data complete */
                    cmdstate = NEWSPIDATATOPROCESS;
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
                    SSC2_SendWord(rxtxbuffer.dataarray_HALL[sendcounter]);
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
                rxtxbuffer.dataarray_HALL[receivecounter] = receivedword;
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
