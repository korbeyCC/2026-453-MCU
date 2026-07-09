#include "app_Data.h"
#include "mid_FLASH.h"
#include "FreeRTOS.h"
#include "task.h"

APP_DATA_HandleTypeDef app_data;
uint8_t EEPROMSet = 0;

void APP_Data_Init(void)
{
    uint32_t addr = MID_FLASH_AddressTransition(0);
    MID_FLASH_ReadData(&addr, sizeof(APP_DATA_HandleTypeDef), (uint16_t *)&app_data);
    
    // 如果 Flash 尚未初始化
    if (app_data.min_height == 0xFFFF)
    {
        app_data.min_height = 1000;
        app_data.max_height = 3000;
        app_data.current_height = 1000;
        APP_Data_Storage();
    }
}

void APP_Data_Storage(void)
{
    taskENTER_CRITICAL();
    
    uint32_t base_addr = MID_FLASH_AddressTransition(0);
    uint32_t target_addr = base_addr;
    
    MID_FLASH_Unlock();
    MID_FLASH_ErasePage(base_addr);
    MID_FLASH_WrithData(&target_addr, sizeof(APP_DATA_HandleTypeDef), (uint16_t *)&app_data);
    MID_FLASH_Lock();
    
    taskEXIT_CRITICAL();
}

void APP_Data_Task(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while (1)
    {
        if (EEPROMSet == 1)
        {
            APP_Data_Storage();
            EEPROMSet = 0;
        }
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(200));
    }
}
