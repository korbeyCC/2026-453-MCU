#ifndef __MID_FLASH_H
#define __MID_FLASH_H

#include "mid_main.h"

void MID_FLASH_Unlock(void);
void MID_FLASH_Lock(void);
void MID_FLASH_ErasePage(uint32_t Address);
uint32_t MID_FLASH_AddressTransition(uint8_t Add_ID);
void MID_FLASH_WrithData(uint32_t *Address, uint16_t Length, uint16_t *Data);
void MID_FLASH_ReadData(uint32_t *Address, uint16_t Length, uint16_t *Data);

#endif
