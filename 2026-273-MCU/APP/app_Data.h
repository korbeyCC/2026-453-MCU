#ifndef __APP_DATA_H
#define __APP_DATA_H

#include "app_main.h"

// 驱动器硬件参数与开机配置基准
#define DRIVER_RATED_STALL_CURRENT_DECI_A 750 // 驱动器额定堵转电流基准: 7.50A (单位: 0.01A)
#define DRIVER_INIT_STALL_CURRENT_PERCENT 150 // 开机 485 配置百分比: 150% (对应 F07.12 寄存器值 1500)
#define DRIVER_FAULT_AUTO_RESET_TIME      50  // F07.09: 故障自动复位间隔 5.0s (单位 0.1s, 写入 50，防机械频繁冲击)
#define DRIVER_FAULT_AUTO_RESET_TIMES     10  // F07.10: 故障自动复位次数 (配置 10 次自动重试自愈)
#define DRIVER_485_TIMEOUT_TIME_VAL       2   // F08.04: 485 通信超时故障时间 0.2s (单位 0.1s, 写入 2，200ms 极速停机)
#define DRIVER_485_TIMEOUT_ACTION         0   // F08.05: 485 传输错误处理 (0: 报警并停机，实测固件仅支持 0)

// 主控堵转电流阈值可调范围 (单位: 0.01A)
#define STALL_CURRENT_THRESHOLD_MIN 50                                                                              // 下限: 0.50A
#define STALL_CURRENT_THRESHOLD_MAX ((DRIVER_RATED_STALL_CURRENT_DECI_A * app_data.driver_stall_percent) / 100) // 上限联动驱动器限流

// 掉电保存高度行程等数据包结构体 (32位对齐)
typedef struct
{
    int32_t min_mount_halls[4];       // 4路立柱安装起点高度绝对霍尔计数
    int32_t motor_abs_halls[4];       // 4路立柱当前运行绝对霍尔位置计数
    int32_t max_travel_range_mm;      // 升降总行程范围 (单位: mm，默认 2000)  1
    uint16_t reduction_ratio;         // 减速比 (默认 30)
    uint16_t target_speed_mm_min;     // 整体运行速度 (单位: mm/min，默认 600) 2
    uint16_t stall_current_threshold; // 堵转电流阈值 (单位: 0.01A，默认 750)  5
    uint16_t max_sync_diff_mm;        // 最大同步差阈值 (单位: mm，默认 5) 4
    uint16_t lead_mm;                 // 丝杆导程 (单位: mm，默认 8)
    uint16_t hall_coef;               // 霍尔系数 (默认 30)
    uint16_t single_tune_step_mm;     // 单轴微调步进距离 (单位: 1mm，默认 1mm，4路共享) - 对应 P1 菜单 q8
    uint16_t motor_dir_invert;        // 丝杆运动方向极性 (0: 默认正向, 1: 极性反转，默认 1) - 对应 P1 菜单 q3
    uint16_t rebound_travel_mm;       // 堵转反弹行程 (单位: mm，默认 1000mm = 1米) - 对应 P1 菜单 q9
    uint16_t column_mode;             // 柱体模式: 0: 4柱(00), 12, 13, 14, 23, 24, 34 - 对应 P1 菜单 q10
    uint16_t column_mode_locked;      // 柱体模式单向锁定状态: 0: 未锁定 (出厂态自由任选), 1: 已锁定 (不可切回00)
    uint16_t show_current_mode;       // 显示模式: 0: 仅位置, 1: 仅实时电流, 2: 8项综合监测(4位置+4现场电流) - 对应 P1 菜单 q11
    uint16_t driver_stall_percent;    // 驱动器堵转限流百分比 (50 ~ 250%, 默认 150%) - 对应 P1 菜单 q12
    uint16_t stall_detect_time_ms;    // 堵转判定防抖时间 (单位: ms, 100 ~ 2000ms, 默认 600ms) - 对应 P1 菜单 q13
    uint16_t reserved;                // 预留对齐 64 字节
} APP_DATA_HandleTypeDef;

extern APP_DATA_HandleTypeDef app_data;
extern uint8_t EEPROMSet;

void APP_Data_Init(void);
void APP_Data_Storage(void);
void APP_Data_ResetDefault(void);
void APP_Data_Task(void *pvParameters);
uint8_t App_Data_GetColumnMotorMask(void);

#endif
