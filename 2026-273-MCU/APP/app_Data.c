#include "app_Data.h"
#include "mid_FLASH.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_Comm.h"
#include <string.h>

APP_DATA_HandleTypeDef app_data;
uint8_t EEPROMSet = 0;

/**
 * @brief  初始化参数系统，读出 Flash 记忆值并进行未初始化检测
 */
void APP_Data_Init(void)
{
    uint32_t addr = MID_FLASH_AddressTransition(0);
    MID_FLASH_ReadData(&addr, sizeof(APP_DATA_HandleTypeDef), (uint16_t *)&app_data);
    
    // 如果 Flash 尚未初始化 (如新烧录固件或 Flash 空白，读取值通常为 0xFFFFFFFF 或 0)
    if (app_data.min_mount_halls[0] == -1 || 
        app_data.min_mount_halls[0] == 0x7FFFFFFF || 
        app_data.max_travel_range <= 0)
    {
        // 初始安装起点高度默认设为 18000 霍尔计数 (对应约 1.2m 起始点，1200mm * 15hall/mm = 18000)
        for (int i = 0; i < 4; i++)
        {
            app_data.min_mount_halls[i] = 18000;
            app_data.motor_abs_halls[i] = 18000;
        }
        
        // 默认可升降总行程设为 25500 霍尔计数 (对应 1.7m 行程，1700mm * 15hall/mm = 25500)
        app_data.max_travel_range = 25500;
        
        // 立即写入出厂默认值进行固化
        APP_Data_Storage();
    }
    
    // 强制使能 PVD 检测及中断 (最高 PVD 检测阈值 2.9V，确保掉电时以最高优先级抢先处理)
    PWR_PVDTypeDef getConfigPVD;
    getConfigPVD.PVDLevel = PWR_PVDLEVEL_7; 
    getConfigPVD.Mode     = PWR_PVD_MODE_IT_RISING; // 电压跌落至阈值以下触发中断
    HAL_PWR_ConfigPVD(&getConfigPVD);
    HAL_PWR_EnablePVD();
    
    // // 配置 EXTI Line 16 PVD 中断抢占优先级并使能
    // HAL_NVIC_SetPriority(PVD_IRQn, 0, 0); // 抢占优先级 0 (断电紧急保护，特高优先级)
    // HAL_NVIC_EnableIRQ(PVD_IRQn);
}

/**
 * @brief  立即同步将参数保存至 Flash 芯片中 (需保证临界区安全)
 */
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

/**
 * @brief  系统设置修改备份守护任务 (EEPROMSet 置 1 时触发，主要用于菜单/用户微调保存)
 */
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

/**
 * @brief  STM32 PWR PVD programmable voltage detector interrupt callback
 * @note   在单片机外部电源掉电瞬间触发，利用电容余电紧急将 4 路电机的绝对霍尔高度持久化归档至 Flash
 */
void HAL_PWR_PVDCallback(void)
{
    // 1. 紧急归档当前内存中最实时精确的立柱绝对高度
    for (int i = 0; i < 4; i++)
    {
        app_data.motor_abs_halls[i] = g_motor_status[i].current_abs_hall;
    }
    
    // 2. 紧急执行 Flash 存储擦写 (利用约 20ms 电容维持供电期快速写入)
    APP_Data_Storage();
}
