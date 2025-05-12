
#ifndef AUTO_ADDRESS_H
#define AUTO_ADDRESS_H

#define ADDRESS_1 4500 //5000mV expected
#define ADDRESS_2 2000 //2500mV expected
#define ADDRESS_3 1500 //1666mV expected
#define ADDRESS_4 500 //1250mV expected

/* Original
#define ADDRESS_1 4500 //5000mV - 10%
#define ADDRESS_2 1500 //2500mV - 10%
#define ADDRESS_3 900 //1666mV - 10%
#define ADDRESS_4 500 //1250mV - 10%
*/

#define VDDP 5000
#define DELTA 200

#define MAX_BUS_CLIENTS 4

struct Auto_Addressing
{
	uint8 My_Address;
	uint16 Test_Address_mV[4]; 
	uint16 Test_Address;
	uint16 Bus_mV;
};

uint8 init_Auto_Addressing(void);
uint8 Auto_Addressing_Handler(void);
void Address_2_Ports(uint16 Address);
void Address_Line_Open(void);
void Address_Line_GND(void);

#endif
