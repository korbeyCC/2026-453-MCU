#include "app_main.h"

void APP_Init(void)
{
    MID_Init();
    APP_Data_Init(); // 载入Flash保存的高度参数

    // 创建显示任务
    xTaskCreate(APP_ShowTask, "APP_Show", 128, NULL, 1, NULL);
    // 创建虚拟按键扫描任务
    xTaskCreate(MID_Key_ScanTask, "MID_KeyScan", 128, NULL, 2, NULL);
    // 创建参数持久化守护任务
    xTaskCreate(APP_Data_Task, "APP_Data", 128, NULL, 2, NULL);
    // 创建菜单设置任务
    xTaskCreate(APP_MenuTask, "APP_Menu", 128, NULL, 2, NULL);
    // 创建外接信号扫描任务
    xTaskCreate(MID_Signal_ScanTask, "MID_SigScan", 128, NULL, 2, NULL);
    // 创建核心业务控制逻辑任务
    xTaskCreate(APP_CtllogicTask, "APP_Ctllogic", 128, NULL, 2, NULL);
    // 创建485串口并行通信超时管理任务
    xTaskCreate(APP_CommTask, "APP_Comm", 128, NULL, 3, NULL);
}
