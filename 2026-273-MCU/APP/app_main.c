#include "app_main.h"
#include <stdio.h>
#include <stdarg.h>
#include "semphr.h"
#include "task.h"

// 导入调试串口 5 句柄
extern UART_HandleTypeDef huart5;

long test_APP[7];
static SemaphoreHandle_t xDebugMutex = NULL;

/**
 * @brief  调试串口 5 格式化输出（线程安全互斥保护）
 */
void Debug_Printf(const char *format, ...)
{
    if (xDebugMutex == NULL && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xDebugMutex = xSemaphoreCreateMutex();
    }
    
    char buffer[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (len > 0) {
        if (xDebugMutex != NULL && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            if (xSemaphoreTake(xDebugMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                HAL_UART_Transmit(&huart5, (uint8_t *)buffer, len, 50);
                xSemaphoreGive(xDebugMutex);
            }
        } else {
            HAL_UART_Transmit(&huart5, (uint8_t *)buffer, len, 50);
        }
    }
}

void APP_Init(void)
{
    MID_Init();
    APP_Data_Init(); // 载入Flash保存的高度参数

    // 创建显示任务
    test_APP[0] = xTaskCreate(APP_ShowTask, "APP_Show", 128, NULL, 1, NULL);
    // 创建虚拟按键扫描任务
    test_APP[1] = xTaskCreate(MID_Key_ScanTask, "MID_KeyScan", 128, NULL, 2, NULL);
    // 创建参数持久化守护任务
    test_APP[2] = xTaskCreate(APP_Data_Task, "APP_Data", 128, NULL, 2, NULL);
    // 创建菜单设置任务
    test_APP[3] = xTaskCreate(APP_MenuTask, "APP_Menu", 128, NULL, 2, NULL);
    // 创建外接信号扫描任务
    test_APP[4] = xTaskCreate(MID_Signal_ScanTask, "MID_SigScan", 128, NULL, 2, NULL);
    // 创建核心业务控制任务
    test_APP[5] = xTaskCreate(APP_ControlTask, "APP_Control", 768, NULL, 2, NULL);
    // 创建485串口并行通信超时管理任务
    test_APP[6] = xTaskCreate(APP_CommTask, "APP_Comm", 256, NULL, 3, NULL);
}
