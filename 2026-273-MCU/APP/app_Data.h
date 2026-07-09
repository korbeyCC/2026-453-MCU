#ifndef __APP_DATA_H
#define __APP_DATA_H

#include "app_main.h"

// 掉电保存高度行程等数据包结构体 (uint16_t 半字自然对齐)
typedef struct
{
    uint16_t min_height;     // 最低安装高度
    uint16_t max_height;     // 最大升降高度
    uint16_t current_height; // 当前高度
    uint16_t reserved[13];   // 预留空间对齐
} APP_DATA_HandleTypeDef;

extern APP_DATA_HandleTypeDef app_data;
extern uint8_t EEPROMSet;

void APP_Data_Init(void);
void APP_Data_Storage(void);
void APP_Data_Task(void *pvParameters);

#endif
