
#include "commonMC.h"
#include "common.h"
#include "tle_device.h"

void changeMode(uint8 reqMode)
{
	WDT1_Service();
	PMU->GPUDATA00.bit.DATA0 = reqMode;
	__NVIC_SystemReset();
}

void checkErrorsMC(uint16* motorStartError)
{
	if(*motorStartError == ERR_NONE) sendAnswer(0x1C00); // TODO: replace with defines
	else sendAnswer(*motorStartError);
	
	*motorStartError = ERR_NONE;
}

// CRC-8 - based on the CRC8 formulas by Dallas/Maxim
// code released under the terms of the GNU GPL 3.0 license
// code from http://www.leonardomiliani.com/en/2013/un-semplice-crc8-per-arduino/
uint8 CRC8(uint8 *data, uint8 len) 
{
	uint8_t crc = 0x55;
  while (len--)
	{
    uint8_t i;
		uint8_t extract = *data++;
    for (i = 8; i; i--) 
		{
      uint8_t sum = (crc ^ extract) & 0x01; 
      crc >>= 1;
      if (sum) crc ^= 0x8C;
      extract >>= 1;
    }
  }
  return crc;
}

uint8 isValueInArray(uint8 val, const uint8 *indices16bit, const uint8 size)
{
	int i;
	for (i = 0; i < size; i++) if (indices16bit[i] == val) return 1;
	return 0;
}
