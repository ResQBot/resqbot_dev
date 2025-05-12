#ifndef COMMON_H
#define COMMON_H

#include "Types.h"

// LEDs
#define LED_R		0x0002
#define LED_G		0x0003
#define LED_B		0x0013

// states for interrupt
#define NEWCOMMAND 0
#define DATA_1_OF_1 1
#define DATA_1_OF_2 2
#define DATA_2_OF_2 3
#define MOTORSPEED 4
#define SENDDATA 5
#define RECEIVEDATA 6

// states for communication main <--> interrupt
#define UNINITIALIZED				0
#define READYFORSPIDATA 		1
#define NEWSPIDATATOPROCESS	2
#define PROCESSINGSPIDATA 	3

// Allgemein
#define CONFIRM_OFFSET 0x1000

// 0x01: modeControl
#define MODECONTROL 0x01
#define GETMYMODE 0x10

// 0x02: loadDataset
#define LOADDATASET 0x02
#define DATASET0 0x00
#define DATASET1 0x01
#define DATASET2 0x02
#define DATASET3 0x03
#define CUSTOMDATASET0 0x10
#define CUSTOMDATASET1 0x11
#define CUSTOMDATASET2 0x12
#define CUSTOMDATASET3 0x13

// 0x03: sendDataset
#define SENDDATASET 0x03

// 0x04: receiveDataset
#define RECEIVEDATASET 0x04

// 0x05: changeParameter
#define CHANGEPARAMETER 0x05

// 0x06: saveDataset
#define SAVEDATASET 0x06
#define POSITION0 0
#define POSITION1 1
#define POSITION2 2
#define POSITION3 3
#define OFFSET_DATASET0  0x00000000
#define OFFSET_DATASET1  0x00000080
#define OFFSET_DATASET2  0x00000100
#define OFFSET_DATASET3  0x00000180

// 0x07: setMotorspeed
#define SETMOTORSPEED 0x07

// 0x08: motorControl
#define MOTORCONTROL 0x08
#define START_MOTOR 0x00
#define STOP_MOTOR 0x01

// 0x09: boardControl
#define BOARDCONTROL 0x09
#define AUTOADDRESSING 0x00
#define BOARD_AVAILABLE 0x01
#define RESET 0x02

// 0x0A: LEDOn
#define LED_ON 0x0A

// 0x0B: LEDOff
#define LED_OFF 0x0B

// 0x0C: Check for error
#define CHECK_ERROR	0x0C
#define ERR_NONE 0x0000

// 0x0F: GetMotorSpeed
#define GETMOTORSPEED 0x0F

// 0x30: readHallPattern
#define READHALLPATTERN 0x55

// 0x11: CHECKSUCCESS
#define CHECKSUCCESS 0x11

#define SUCCESS 0x1111

#define CHECKFAIL 0x1010



void sendPosAnswer(uint16 msg);
void sendAnswer(uint16 msg);

bool checkCommand(uint16 msg, uint8 command, uint8 param);
void setBoardnr(uint8 boardnr);
void startAutoaddressing(void);

void selectSlave(void);
void deselectSlave(void);

#endif
