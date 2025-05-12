
#include "commands.h"
#include "tle_device.h"
#include "Emo_RAM.h"
#include "globvars_FOC.h"
#include "..\..\auto_address.h" 
#include "..\..\common.h"
#include "..\..\commonMC.h"

//--------------------0x02: loadDataset--------------------
void loadDataset(uint8 datasetnr)
{
    switch (datasetnr) 
    {
        case DATASET0: Emo_Focpar_Cfg = dataset0; break;
        case DATASET1: Emo_Focpar_Cfg = dataset1; break;
        case DATASET2: Emo_Focpar_Cfg = dataset2; break;
        case DATASET3: Emo_Focpar_Cfg = dataset3; break;
        case CUSTOMDATASET0: Emo_Focpar_Cfg = customdataset0; break;
        case CUSTOMDATASET1: Emo_Focpar_Cfg = customdataset1; break;
        case CUSTOMDATASET2: Emo_Focpar_Cfg = customdataset2; break;
        case CUSTOMDATASET3: Emo_Focpar_Cfg = customdataset3; break;
        default: break;
    }
    Emo_update();
}

//--------------------0x05: changeParameter--------------------
void changeSingleParameter(uint8 index, uint16 data1, uint16 data2)
{
    rxtxbuffer.datastruct_FOC = Emo_Focpar_Cfg;
    if (isValueInArray(index, indices_16bit, indices_16bit_size) == 1) // 16bit variable
    {	
        rxtxbuffer.dataarray_FOC[index] = data1;
    }
    else // 32bit var 
    {
        rxtxbuffer.dataarray_FOC[index] = data2;
        rxtxbuffer.dataarray_FOC[index + 1] = data1;
    }
    Emo_Focpar_Cfg = rxtxbuffer.datastruct_FOC;
    Emo_update();
}

//--------------------0x06: saveCurrentDataset--------------------
uint8 saveCurrentDataset(uint8 position)
{
        uint8 erase_error = 0, writeprotection_off = 0, checksumRAM = 0, checksumROM = 0;
        uint32 flashpageStart = 0;
        uint8 *ptrToROMData;
        uint8 *ptrToRAMData = (uint8*)&rxtxbuffer;
        rxtxbuffer.datastruct_FOC = Emo_Focpar_Cfg;
        
        switch (position)
        {
            case POSITION0: flashpageStart = STARTOFCUSTOMDATASETS + OFFSET_DATASET0; break;
            case POSITION1: flashpageStart = STARTOFCUSTOMDATASETS + OFFSET_DATASET1; break;
            case POSITION2: flashpageStart = STARTOFCUSTOMDATASETS + OFFSET_DATASET2; break;
            case POSITION3: flashpageStart = STARTOFCUSTOMDATASETS + OFFSET_DATASET3; break;
            default: return 1;
        }

        // erase page
        writeprotection_off = USER_CFLASH_WR_PROT_DIS((uint16) 0);
        if(writeprotection_off == 0)
        {
            erase_error = USER_ERASEPG((uint32*)flashpageStart, 0);
        
            if (erase_error == 0u)
            {
                // write data
                uint8 i;
                checksumRAM = CRC8(ptrToRAMData, NUMBEROF_BYTES_FOC);
                ptrToROMData = (uint8*)flashpageStart;
                
                WDT1_SOW_Service((uint8)1);
                USER_OPENAB((uint32*)flashpageStart);
                WDT1_Service();
                
                for (i = 0; i < NUMBEROF_BYTES_FOC; i++) ptrToROMData[i] = ptrToRAMData[i];
                
                WDT1_SOW_Service((uint8)1);
                USER_PROG(0);
                WDT1_Service();
                
                checksumROM = CRC8(ptrToROMData, NUMBEROF_BYTES_FOC);
                if(checksumRAM == checksumROM) return 0;
                else return 2;
            }
        }
        return 3;
}

//--------------------other functions--------------------
// updateDatasetDependencies
void updateDatasetDependencies()
{
        /* FOC_PWM_FREQ  */
        ccu6_t12_freq = Emo_Focpar_Cfg.PWM_Frequency;
        ccu6_t12_period_sel = 2; /* constant */
        pwm_period = 1e6/ccu6_t12_freq;
        ccu6_tctr0_bit7 = (CCU6_TCTR0 & 0x0080) >> 7; /* constant */
        ccu6_t12period = ccu6_t12_period_sel<1?CCU6_T12_TICK:ccu6_t12_period_sel<2?CCU6_T12_CLK*CCU6_T12_TIME/(1+1*ccu6_tctr0_bit7):((CCU6_T12_CLK*1e6)/ccu6_t12_freq)/(1+1*ccu6_tctr0_bit7);
        ccu6_t12pr = ccu6_t12_period_sel<1?CCU6_T12_TICK-1:ccu6_t12_period_sel<2?CCU6_T12_CLK*CCU6_T12_TIME/(1+1*ccu6_tctr0_bit7)-1:((CCU6_T12_CLK*1e6)/ccu6_t12_freq)/(1+1*ccu6_tctr0_bit7)-1;
        ccu6_cc60sr = CCU6_CH0_CMP_TICK+(CCU6_T12_CLK*CCU6_CH0_CMP_TIME)+(ccu6_t12pr*CCU6_CH0_CMP_DC/100);
        ccu6_cc61sr = CCU6_CH1_CMP_TICK+(CCU6_T12_CLK*CCU6_CH1_CMP_TIME)+(ccu6_t12pr*CCU6_CH1_CMP_DC/100);
        ccu6_cc62sr = CCU6_CH2_CMP_TICK+(CCU6_T12_CLK*CCU6_CH2_CMP_TIME)+(ccu6_t12pr*CCU6_CH2_CMP_DC/100);
            
        CCU6_Init();
}

