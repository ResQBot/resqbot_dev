#ifndef _COMMANDS_H
#define _COMMANDS_H

#include "types.h"
#include "stdint.h"

// for 0x04 receiveDataset
#define NROFMESSAGES 22
#define NROFBYTES NROFMESSAGES*2

// 0x06: saveDataset
#define STARTOFCUSTOMDATASETS 0x1101D000

void loadDataset(uint8 word1byte2);
void changeSingleParameter(uint8 index, uint16 data_part1, uint16 data_part2);
uint8 saveCurrentDataset(uint8 position);

void updateDatasetDependencies(void);
void init_emoccucfg(uint8 offset60deg_en);

/**
 * \brief Reads sampled Hall pattern from CCU6 CMPSTAT register.
 * \return Sampled Hall pattern
 * \author Beatrix Dietl
 */
uint32 readHallPattern(void);

#endif


