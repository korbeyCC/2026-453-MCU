#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "app_main.h"

/* 系统整体运行流程状态机 */
typedef enum {
    SYS_STEP_Boot = 0,               // 系统刚上电引导启动（等待硬件初始化）
    SYS_STEP_READY,                  // 系统配置完成，就绪待命（等待输入）

    SYS_STEP_SINGLE_TUNE,            // 单路立柱微调控制中
    SYS_STEP_TUNE_DONE,              // 微调结束 (Flash 归档中)
    
    SYS_STEP_TOTAL_FORWARD,          // 同步上升起跑段
    SYS_STEP_TOTAL_REVERSE,          // 同步下降起跑段
    SYS_STEP_TOTAL_RUNNING,          // 整体运行调速阶段（定频 PID 泵）
    SYS_STEP_TOTAL_DONE,             // 整体结束停机中 (Flash 归档中)
    SYS_STEP_FAULT_STOP              // 故障急停状态
} Motor_ctl_Step_t;

/* 电机单通道运行与状态监控 */
typedef struct {
    int32_t base_abs_hall;       // 本次运行起步前的绝对高度基准 (有符号)
    int32_t start_drive_hall;    // 本次起步时驱动器的原始读数起点 (有符号)
    int32_t current_abs_hall;    // 实时解算的绝对高度 (有符号，用于 PID 控制)
    uint32_t hall_value;         // 驱动器最新原始无符号霍尔读数 (内存数据缓存)
    uint16_t current_deciA;      // 驱动器当前输出电流 (单位: 0.01A)
    uint8_t stall_cnt;           // 过流堵转判定计数器
    uint8_t comm_error;          // 通信超时错误标记
    uint8_t retry_cnt;           // 重试计数
    
    uint8_t target_cmd;          // 目标动作指令
    int16_t target_speed;        // 目标转速
    uint8_t current_cmd;         // 驱动器当前真实的运行指令
    int16_t current_speed;       // 驱动器当前真实的转速配置
} Motor_Status_t;

/* 全局控制上下文 (解耦数据驱动架构) */
typedef struct {
    Motor_ctl_Step_t system_step;       // 系统级运行状态机
    Motor_Status_t g_motor_status[4];   // 4路立柱电机状态快照
    volatile bool is_hardware_ready;    // 4路 Modbus 硬件初始化完成标志
    int16_t base_speed;                 // 系统全局基准转速 (RPM)
    
    // 自动物理换算参数与安防状态
    uint32_t counts_per_mm;             // 每 mm 霍尔计数值
    int32_t max_sync_diff_hall;         // 最大同步差霍尔计数值
    int32_t max_travel_hall;            // 最大行程霍尔计数值
    int16_t calc_base_rpm;              // 算出的基准 RPM
    uint8_t system_fault_code;          // 故障代码 (0:正常, 1:过流堵转, 2:通信中断, 3:同步差超限)
} Sys_Ctrl_Context_t;

// 全局外部变量声明
extern Sys_Ctrl_Context_t g_sys_context;

void APP_ControlTask(void *pvParameters);

#endif // __APP_CONTROL_H
