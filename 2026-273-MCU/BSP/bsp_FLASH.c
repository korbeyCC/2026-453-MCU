#include "bsp_FLASH.h"

void BSP_FLASH_Unlock(void)
{
    HAL_FLASH_Unlock();
}

void BSP_FLASH_Lock(void)
{
    HAL_FLASH_Lock();
}

/**
 * @brief  擦除一页（扇区）
 * @note   调用前需确保 Flash 已解锁（BSP_FLASH_Unlock）
 */
void BSP_FLASH_ErasePage(uint32_t Address)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = Address;
    EraseInitStruct.NbPages = 1;

    HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
}

void BSP_FLASH_WriteHALFWORD(uint32_t Address, uint16_t Data)
{
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, Data);
}

uint16_t BSP_FLASH_ReadHALFWORD(uint32_t Address)
{
    return *(__IO uint16_t*)Address;
}
