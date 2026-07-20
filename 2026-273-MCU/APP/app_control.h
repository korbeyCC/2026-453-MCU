#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "app_main.h"
#include "event_groups.h" // 导入 FreeRTOS 事件标志组支持

// ========================== FreeRTOS 事件标志位定义 ==========================
#define CMD_EVENT_BIT(motor_idx)      (1 << (motor_idx))        // 电机 0~3 写控制寄存器完成 (Bit0~Bit3)
#define SPEED_EVENT_BIT(motor_idx)    (1 << ((motor_idx) + 4))  // 电机 0~3 写速度寄存器完成 (Bit4~Bit7)
#define READ_EVENT_BIT(motor_idx)     (1 << ((motor_idx) + 8))  // 电机 0~3 读霍尔寄存器返回 (Bit8~Bit11)

// 4路立柱均集齐的事件掩码值
#define ALL_CMD_EVENTS_READY          0x000F  // Bit0~Bit3 均为 1
#define ALL_SPEED_EVENTS_READY        0x00F0  // Bit4~Bit7 均为 1
#define ALL_READ_EVENTS_READY         0x0F00  // Bit8~Bit11 均为 1

/* 系统整体运行流程状态机 */
typedef enum {
    SYS_STEP_Boot = 0,               // 系统刚上电引导启动与其它初始化
    
    // 全局 5 步 Modbus 初始化流程
    SYS_STEP_INIT_WRITE_ENABLE,      // 0x200E 写使能选定 (发送并等4路回应)
    SYS_STEP_INIT_RUN_MODE,          // 0x2006 运行模式配置 (发送并等4路回应)
    SYS_STEP_INIT_SPEED_MODE,        // 0x2007 速度模式配置 (发送并等4路回应)
    SYS_STEP_INIT_SET_SPEED,         // 0x2001 初始化转速写入 (发送并等4路回应)
    SYS_STEP_INIT_START_RUN,         // 0x2000 驱动器启动使能 (发送并等4路回应)
    
    SYS_STEP_READY,                  // 系统上电配置完毕，就绪空闲 (等待用户按键)

    SYS_STEP_SINGLE_TUNE,            // 单路立柱微调控制中
    SYS_STEP_TUNE_DONE,              // 微调结束 (末帧读取与 Flash 归档中)
    
    SYS_STEP_TOTAL_FORWARD,          // 同步上升起跑段 (下发速度/方向并等待双应答)
    SYS_STEP_TOTAL_REVERSE,          // 同步下降起跑段 (下发速度/方向并等待双应答)
    SYS_STEP_TOTAL_RUNNING,          // 整体运行调速阶段 (流水线霍尔读取与事件 PID 纠偏)
    SYS_STEP_TOTAL_DONE              // 整体结束停机中 (末帧读取与 Flash 归档中)
} Motor_ctl_Step_t;

/* 电机单通道运行与状态监控 */
typedef struct {
    int32_t base_abs_hall;       // 本次运行起步前的绝对高度基准 (有符号)
    int32_t start_drive_hall;    // 本次起步时驱动器的原始读数起点 (有符号)
    int32_t current_abs_hall;    // 实时解算的绝对高度 (有符号，用于 PID 控制)
    uint32_t hall_value;         // 驱动器最新原始无符号霍尔读数 (起步锁存基准)
    uint8_t comm_error;          // 通信超时错误标记
    uint8_t retry_cnt;           // 重试计数
    
    uint8_t target_cmd;          // 目标动作指令
    int16_t target_speed;        // 目标转速
    uint8_t current_cmd;         // 驱动器当前真实的运行指令
    int16_t current_speed;       // 驱动器当前真实的转速配置
} Motor_Status_t;

/* 全局控制上下文 (使用 FreeRTOS 事件组管理并发状态同步) */
typedef struct {
    Motor_ctl_Step_t system_step;       // 系统级运行状态机
    Motor_Status_t g_motor_status[4];   // 4路立柱电机状态快照
    EventGroupHandle_t event_group;     // FreeRTOS 事件标志组句柄
    int16_t base_speed;                 // 系统全局基准转速 (RPM)
} Sys_Ctrl_Context_t;

// 全局外部变量声明
extern Sys_Ctrl_Context_t g_sys_context;

void APP_ControlTask(void *pvParameters);

#endif // __APP_CONTROL_H
