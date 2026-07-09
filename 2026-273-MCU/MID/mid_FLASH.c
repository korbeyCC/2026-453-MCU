#include "mid_FLASH.h"
#include "bsp_FLASH.h"

void MID_FLASH_Unlock(void)
{
    BSP_FLASH_Unlock();
}

void MID_FLASH_Lock(void)
{
    BSP_FLASH_Lock();
}

void MID_FLASH_ErasePage(uint32_t Address)
{
    BSP_FLASH_ErasePage(Address);
}

/* 根据 PageID 返回对应的物理 Flash 页面绝对地址 */
uint32_t MID_FLASH_AddressTransition(uint8_t Add_ID)
{
    return FLASH_SAVE_ADDR - Add_ID * STM32_SECTOR_SIZE;
}

void MID_FLASH_WrithData(uint32_t *Address, uint16_t Length, uint16_t *Data)
{
    while (Length)
    {
        BSP_FLASH_WriteHALFWORD((*Address), (*Data));
        (*Address) += 2;
        Data++;
        Length = (Length >= 2) ? (Length - 2) : 0;
    }
}

void MID_FLASH_ReadData(uint32_t *Address, uint16_t Length, uint16_t *Data)
{
    while (Length)
    {
        (*Data) = BSP_FLASH_ReadHALFWORD((*Address));
        (*Address) += 2;
        Data++;
        Length = (Length >= 2) ? (Length - 2) : 0;
    }
}
