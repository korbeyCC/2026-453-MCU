#ifndef __APP_DATA_H
#define __APP_DATA_H

#include "app_main.h"

// 掉电保存高度行程等数据包结构体 (32位对齐)
typedef struct
{
    int32_t  min_mount_halls[4];  // 4路立柱安装起点高度绝对霍尔计数
    int32_t  motor_abs_halls[4];  // 4路立柱当前运行绝对霍尔位置计数
    int32_t  max_travel_range;    // 升降总行程范围霍尔计数值
    uint16_t reserved[12];        // 预留空间对齐
} APP_DATA_HandleTypeDef;

extern APP_DATA_HandleTypeDef app_data;
extern uint8_t EEPROMSet;

void APP_Data_Init(void);
void APP_Data_Storage(void);
void APP_Data_Task(void *pvParameters);

#endif
