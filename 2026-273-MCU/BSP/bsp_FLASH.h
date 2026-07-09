#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include "bsp_main.h"

// 所选STM32/CKS32的FLASH容量大小(单位为K)
#define STM32_FLASH_SIZE 64 // 本产品选用的芯片FLASH大小为 64K

#if STM32_FLASH_SIZE < 256
#define STM32_SECTOR_SIZE 1024 // 一页为 1K 字节
#else
#define STM32_SECTOR_SIZE 2048 // 一页为 2K 字节
#endif

#define STM32_FLASH_BASE 0x08000000 // FLASH起始地址

// 写入的FLASH地址，这里为从倒数第一个扇区地址开始写
#define FLASH_SAVE_ADDR (STM32_FLASH_BASE + STM32_SECTOR_SIZE * (STM32_FLASH_SIZE - 1))

void BSP_FLASH_Unlock(void);
void BSP_FLASH_Lock(void);
void BSP_FLASH_WriteHALFWORD(uint32_t Address, uint16_t Data);
uint16_t BSP_FLASH_ReadHALFWORD(uint32_t Address);
void BSP_FLASH_ErasePage(uint32_t Address);

#endif
