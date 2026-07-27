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
    SYS_STEP_TOTAL_REBOUND,          // 堵转后整体反方向反弹阶段
    SYS_STEP_TOTAL_DONE,             // 整体结束停机中 (Flash 归档中)
    SYS_STEP_FAULT_STOP              // 故障急停状态
} Motor_ctl_Step_t;

/* 电机单通道运行与状态监控 */
typedef struct {
    int32_t base_abs_hall;       // 本次运行起步前的绝对高度基准 (有符号)
    uint32_t start_drive_hall;   // 本次起步时驱动器的原始无符号读数起点 (无符号 32 位)
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
    uint16_t ramp_cnt;                  // 300ms 缓启动递增计数器 (0~60)
    
    // 1. 当帧位移增量与其全局统计量共享字段
    float delta_h[4];                   // 4 轴当帧最新的位移增量 ΔH_i
    float avg_delta_h;                  // 4 轴当帧平均位移增量 ΔH_avg
    float max_dh_diff;                  // 4 轴当帧最大轴间增量偏差 (max - min)
    
    // 2. 基于调平零点 (min_mount_halls) 的绝对伸出高度及其统计量 (专用于 PID 绝对纠偏)
    float travel_rel[4];                // 4 轴当帧绝对伸出行程 (counts)
    float avg_travel;                   // 4 轴当帧平均绝对伸出行程 (counts)
    float max_travel_diff;              // 4 轴当帧最大绝对高度差 (max - min)
    
    // 自动物理换算参数与安防状态
    float counts_per_mm;                // 每 mm 霍尔计数值 (支持 112.5f 浮点精度)
    int32_t max_sync_diff_hall;         // 最大同步差霍尔计数值
    int32_t max_travel_hall;            // 最大行程霍尔计数值
    int16_t calc_base_rpm;              // 算出的基准 RPM
    uint8_t system_fault_code;          // 故障代码 (0:正常, 1:过流堵转, 2:通信中断, 3:同步差超限)
    volatile uint32_t hall_update_seq[4];// 4 轴霍尔成功更新打卡序列号

    // 单轴微调与过流反弹控制状态字段
    uint8_t single_tune_dir;            // 微调方向 (0: 正转/上升, 1: 反转/下降)
    uint8_t single_tune_motor_idx;      // 当前微调的目标电机 (0~3)
    uint32_t single_tune_start_hall;    // 微调开始时的驱动器原始霍尔起点
    int32_t single_tune_orig_abs_hall;  // 微调开始时的起点绝对霍尔高度
    uint32_t single_tune_target_counts; // 微调目标霍尔步计数

    uint8_t rebound_cmd;                // 反弹运动指令 (CMD_FORWARD 或 CMD_REVERSE)
    uint32_t rebound_start_hall[4];     // 反弹开始时 4 轴原始霍尔读数
    uint32_t rebound_target_counts;     // 反弹目标霍尔增量计数 (30mm * counts_per_mm)
} Sys_Ctrl_Context_t;

// 全局外部变量声明
extern Sys_Ctrl_Context_t g_sys_context;

void APP_Control_UpdateParamsFromAppData(void);
void APP_Control_StartSingleTune(uint8_t m_idx);
void APP_Control_CancelSingleTune(void);
void APP_ControlTask(void *pvParameters);

#endif // __APP_CONTROL_H
