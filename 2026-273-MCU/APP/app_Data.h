#ifndef __APP_DATA_H
#define __APP_DATA_H

#include "app_main.h"

// 掉电保存高度行程等数据包结构体 (32位对齐)
typedef struct
{
    int32_t min_mount_halls[4];         // 4路立柱安装起点高度绝对霍尔计数
    int32_t motor_abs_halls[4];         // 4路立柱当前运行绝对霍尔位置计数
    int32_t max_travel_range_mm;        // 升降总行程范围 (单位: mm，默认 2000)  1
    uint16_t reduction_ratio;           // 减速比 (默认 30)
    uint16_t target_speed_mm_min;       // 整体运行速度 (单位: mm/min，默认 600) 2
    uint16_t stall_current_threshold;   // 堵转电流阈值 (单位: 0.01A，默认 100)  5
    uint16_t max_sync_diff_mm;          // 最大同步差阈值 (单位: mm，默认 5) 4
    uint16_t lead_mm;                   // 丝杆导程 (单位: mm，默认 8)
    uint16_t hall_coef;                 // 霍尔系数 (默认 30)
    uint16_t single_tune_step_0_1mm;   // 单轴微调步进距离 (单位: 0.1mm，默认 10 = 1.0mm，4路共享)
    uint16_t single_tune_speed_rpm;    // 单轴微调速度 (单位: RPM，默认 100，驱动器原始转速，未算减速比)
    uint16_t reserved[4];              // 预留空间对齐 32 字节
} APP_DATA_HandleTypeDef;

extern APP_DATA_HandleTypeDef app_data;
extern uint8_t EEPROMSet;

void APP_Data_Init(void);
void APP_Data_Storage(void);
void APP_Data_Task(void *pvParameters);

#endif
