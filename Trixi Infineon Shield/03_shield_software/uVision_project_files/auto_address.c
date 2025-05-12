
#include <tle_device.h>
#include <adc1.h>
#include "auto_address.h"

struct Auto_Addressing AA;

uint8 init_Auto_Addressing(void)
{	
    AA.My_Address = 0;
    AA.Bus_mV = 5000;
    AA.Test_Address = 1;
    AA.Test_Address_mV[0] = ADDRESS_1;
    AA.Test_Address_mV[1] = ADDRESS_2;
    AA.Test_Address_mV[2] = ADDRESS_3;
    AA.Test_Address_mV[3] = ADDRESS_4;
    Address_Line_GND();
    
    ADC1_Init();
    VAREF_Enable();

    while(AA.My_Address == 0)
    {
        Auto_Addressing_Handler();
        WDT1_Service();
    }
    
    return AA.My_Address;
}

uint8 Auto_Addressing_Handler(void)
{		
    ADC1_GetChResult_mV(&AA.Bus_mV , ADC1_CH5); // was ADC1_CH3 in original
    
    if(AA.Bus_mV > ADDRESS_4)
    {
        if(AA.Bus_mV > AA.Test_Address_mV[AA.Test_Address - 1])
        {
            AA.My_Address = AA.Test_Address;
            Address_Line_Open();
        }
        else if(AA.Test_Address < MAX_BUS_CLIENTS)
        {
            AA.Test_Address++;
        }
    }
    return true;
}

void Address_2_Ports(uint16 Address)
{
    switch(Address)
    {
        case 1: PORT->P0_DATA.bit.P1 = 1;
                        PORT->P0_DATA.bit.P2 = 0;
                        PORT->P0_DATA.bit.P3 = 0;
                        PORT->P1_DATA.bit.P2 = 0;
                        break;
        case 2: PORT->P0_DATA.bit.P1 = 1;
                        PORT->P0_DATA.bit.P2 = 1;
                        PORT->P0_DATA.bit.P3 = 0;
                        PORT->P1_DATA.bit.P2 = 0;
                        break;
        case 3: PORT->P0_DATA.bit.P1 = 1;
                        PORT->P0_DATA.bit.P2 = 1;
                        PORT->P0_DATA.bit.P3 = 1;
                        PORT->P1_DATA.bit.P2 = 0;
                        break;
        case 4: PORT->P0_DATA.bit.P1 = 1;
                        PORT->P0_DATA.bit.P2 = 1;
                        PORT->P0_DATA.bit.P3 = 1;
                        PORT->P1_DATA.bit.P2 = 1;
                        break;
        default:break;
    }
}

void Address_Line_Open(void)
{
    PORT->P0_DIR.bit.P4 = 0;
    PORT->P0_DATA.bit.P4 = 1;
}

void Address_Line_GND(void)
{
    PORT->P0_DIR.bit.P4 = 1;
    PORT->P0_DATA.bit.P4 = 0;
}
