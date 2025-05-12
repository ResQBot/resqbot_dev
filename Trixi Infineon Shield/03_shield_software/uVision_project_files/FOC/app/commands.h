
#ifndef _COMMANDS_H
#define _COMMANDS_H

#include "types.h"

/*
#define NROFMESSAGES 23
#define NROFBYTES (NROFMESSAGES * 2)
*/

#define STARTOFCUSTOMDATASETS 0x1101E000

void loadDataset(uint8 word1byte2);
void changeSingleParameter(uint8 index, uint16 data_part1, uint16 data_part2);
uint8 saveCurrentDataset(uint8 position);
uint8 loadDatasetFromROM(uint8 position);

void updateDatasetDependencies(void);

#endif
