
#include "commands.h"
#include "tle_device.h"
#include "..\..\auto_address.h" 
#include "globvars_BEMF.h"
#include "Emo_BEMF.h"
#include "..\..\common.h"
#include "..\..\commonMC.h"

//--------------------0x02: loadDataset--------------------
void loadDataset(uint8 datasetnr)
{
    switch (datasetnr) 
    {
        case DATASET0:
            Emo_Bemfpar_Cfg = dataset0.emocfg_BEMF;
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFCFF) | (dataset0.spike_filt << 8);
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFFF8) | dataset0.blank_filt;
            break;
        case DATASET1:
            Emo_Bemfpar_Cfg = dataset1.emocfg_BEMF;
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFCFF) | (dataset1.spike_filt << 8);
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFFF8) | dataset1.blank_filt;
            break;
        case DATASET2:
            Emo_Bemfpar_Cfg = dataset2.emocfg_BEMF;
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFCFF) | (dataset2.spike_filt << 8);
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFFF8) | dataset2.blank_filt;
            break;
        case DATASET3:
            Emo_Bemfpar_Cfg = dataset3.emocfg_BEMF;
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFCFF) | (dataset3.spike_filt << 8);
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFFF8) | dataset3.blank_filt;
            break;
        case CUSTOMDATASET0:
            Emo_Bemfpar_Cfg = customdataset0.emocfg_BEMF;
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFCFF) | (customdataset0.spike_filt << 8);
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFFF8) | customdataset0.blank_filt;
            break;
        case CUSTOMDATASET1:
            Emo_Bemfpar_Cfg = customdataset1.emocfg_BEMF;
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFCFF) | (customdataset1.spike_filt << 8);
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFFF8) | customdataset1.blank_filt;
            break;
        case CUSTOMDATASET2:
            Emo_Bemfpar_Cfg = customdataset2.emocfg_BEMF;
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFCFF) | (customdataset2.spike_filt << 8);
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFFF8) | customdataset2.blank_filt;
            break;
        case CUSTOMDATASET3:
            Emo_Bemfpar_Cfg = customdataset3.emocfg_BEMF;
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFCFF) | (customdataset3.spike_filt << 8);
            MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFFF8) | customdataset3.blank_filt;
            break;
        default:
            break;
    }
    Emo_Update();
}

//--------------------0x05: changeParameter--------------------
void changeSingleParameter(uint8 index, uint16 data1, uint16 data2)
{
    rxtxbuffer.datastruct_BEMF.emocfg_BEMF = Emo_Bemfpar_Cfg;
    
    if (isValueInArray(index, indices_16bit, indices_16bit_size) == 1) // 16bit variable
    {	
        rxtxbuffer.dataarray_BEMF[index] = data1;
    }
    else // 32bit var 
    {
        uint32 *uint32pointer;
        uint32 uint32data = ((uint32)data1 << 16) & 0xFFFF0000;
        uint32data = uint32data | data2;
        
        switch (index)
        {
            case 22: /* BEMF_START_SPEED_PWM_MIN_OFFSET */
                rxtxbuffer.dataarray_BEMF[index] = data2;
                rxtxbuffer.dataarray_BEMF[index+1] = data1;
                rxtxbuffer.datastruct_BEMF.emocfg_BEMF.StartSpeedPwmMin = bemfEndSpeedPwmMin - rxtxbuffer.datastruct_BEMF.emocfg_BEMF.StartSpeedPwmMinOffset;
                break;
            case 29: /* BEMF_END_SPEED_PWM_MIN */
                uint32pointer = (uint32*)&bemfEndSpeedPwmMin;
                *uint32pointer = uint32data;
                rxtxbuffer.datastruct_BEMF.emocfg_BEMF.StartSpeedPwmMin =  bemfEndSpeedPwmMin - rxtxbuffer.datastruct_BEMF.emocfg_BEMF.StartSpeedPwmMinOffset;
                break;
            default: /* BEMF_PWM_FREQ, BEMF_SPEED_FILT_TIME, BEMF_RUNBEGIN_SPEED_PWM_MIN, BEMF_ZERO_VEC_TIME, BEMF_SWITCH_ON_SPEED, BEMF_END_START_SPEED, BEMF_START_ACCEL, BEMF_RUN_SPEED_PWM_MIN, BEMF_RUN_ACCEL */
                rxtxbuffer.dataarray_BEMF[index] = data2;
                rxtxbuffer.dataarray_BEMF[index+1] = data1;
                break;
        }
    }
    Emo_Bemfpar_Cfg = rxtxbuffer.datastruct_BEMF.emocfg_BEMF;
    Emo_Update();
}

//--------------------0x06: saveCurrentDataset--------------------
uint8 saveCurrentDataset(uint8 position)
{
        uint8 erase_error = 0, writeprotection_off = 0, checksumRAM = 0, checksumROM = 0;
        uint32 flashpageStart = 0;
        uint8 *ptrToROMData;
        uint8 *ptrToRAMData = (uint8*)&rxtxbuffer;
        rxtxbuffer.datastruct_BEMF.emocfg_BEMF = Emo_Bemfpar_Cfg;
        
        switch (position)
        {
            case POSITION0:
                flashpageStart = STARTOFCUSTOMDATASETS + OFFSET_DATASET0;
                break;
            case POSITION1:
                flashpageStart = STARTOFCUSTOMDATASETS + OFFSET_DATASET1;
                break;
            case POSITION2:
                flashpageStart = STARTOFCUSTOMDATASETS + OFFSET_DATASET2;
                break;
            case POSITION3:
                flashpageStart = STARTOFCUSTOMDATASETS + OFFSET_DATASET3;
                break;
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
                checksumRAM = CRC8(ptrToRAMData, NROFBYTES);
                ptrToROMData = (uint8*)flashpageStart;
                
                WDT1_SOW_Service((uint8)1);
                USER_OPENAB((uint32*)flashpageStart);
                WDT1_Service();
                
                for (i = 0; i < NROFBYTES; i++)
                {
                    ptrToROMData[i] = ptrToRAMData[i];
                }
                
                WDT1_SOW_Service((uint8)1);
                USER_PROG(0);
                WDT1_Service();
                
                checksumROM = CRC8(ptrToROMData, NROFBYTES);
                if(checksumRAM == checksumROM)
                {
                    return 1;
                }
                else
                {
                    return 0;
                }
            }
        }
        return 0;
}

//--------------------other functions--------------------
// updateDatasetDependencies
void updateDatasetDependencies()
{
        /* BEMF_PWM_FREQ  */
        ccu6_t12_freq = Emo_Bemfpar_Cfg.PWM_Frequency;
        ccu6_t12_period_sel = 2; /* constant */
        pwm_period = 1e6/ccu6_t12_freq;
        ccu6_tctr0_bit7 = (CCU6_TCTR0 & 0x0080) >> 7; /* constant */
        ccu6_t12period = ccu6_t12_period_sel<1?CCU6_T12_TICK:ccu6_t12_period_sel<2?CCU6_T12_CLK*CCU6_T12_TIME/(1+1*ccu6_tctr0_bit7):((CCU6_T12_CLK*1e6)/ccu6_t12_freq)/(1+1*ccu6_tctr0_bit7);
        ccu6_t12pr = ccu6_t12_period_sel<1?CCU6_T12_TICK-1:ccu6_t12_period_sel<2?CCU6_T12_CLK*CCU6_T12_TIME/(1+1*ccu6_tctr0_bit7)-1:((CCU6_T12_CLK*1e6)/ccu6_t12_freq)/(1+1*ccu6_tctr0_bit7)-1;
        ccu6_cc60sr = CCU6_CH0_CMP_TICK+(CCU6_T12_CLK*CCU6_CH0_CMP_TIME)+(ccu6_t12pr*CCU6_CH0_CMP_DC/100);
        ccu6_cc61sr = CCU6_CH1_CMP_TICK+(CCU6_T12_CLK*CCU6_CH1_CMP_TIME)+(ccu6_t12pr*CCU6_CH1_CMP_DC/100);
        ccu6_cc62sr = CCU6_CH2_CMP_TICK+(CCU6_T12_CLK*CCU6_CH2_CMP_TIME)+(ccu6_t12pr*CCU6_CH2_CMP_DC/100);
        
        /* BEMF_START_SPEED_PWM_MIN_OFFSET, BEMF_END_SPEED_PWM_MIN */
        Emo_Bemfpar_Cfg.StartSpeedPwmMin =  bemfEndSpeedPwmMin - Emo_Bemfpar_Cfg.StartSpeedPwmMinOffset;
    
        /* BEMF_SPIKE_FILT and BEMF_BLANK_FILT */
        MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFCFF) | (rxtxbuffer.datastruct_BEMF.spike_filt << 8);
        MF->TRIM_BEMFx.reg = (uint32) (MF->TRIM_BEMFx.reg & 0xFFFFFFF8) | rxtxbuffer.datastruct_BEMF.blank_filt;
    
        CCU6_Init();
}
