#ifndef __APP_COMM_H
#define __APP_COMM_H

#include "app_main.h"
#include "app_control.h" // 导入全局控制状态和结构体

// 驱动器 Modbus 轮询采样配置
#define CURRENT_POLL_INTERVAL_FRAMES 25 // 电流采样帧间隔 (25帧 * 4ms = 100ms 抽样读取 0x3004)

// 核心配置与控制指令字用枚举包装
typedef enum {
    CMD_NONE = 0,
    CMD_FORWARD,           // 1: 启动正转 (0x0001)
    CMD_REVERSE,           // 2: 启动反转 (0x0002)
    CMD_STOP,              // 3: 刹车停机 (0x0009)
    CMD_INIT_WRITE_ENABLE, // 4: 0x200E 写使能
    CMD_INIT_RUN_MODE,     // 5: 0x2006 运行方式选择
    CMD_INIT_SPEED_MODE,   // 6: 0x2007 速度命令选择
    CMD_SET_SPEED,         // 7: 0x2001 目标速度设置
    CMD_INIT_START_RUN,    // 8: 0x2000 普通控制指令
    CMD_READ_HALL,         // 9: 周期读取霍尔高度 (0x3013, 2 regs)
    CMD_READ_CURRENT,      // 10: 周期读取输出电流 (0x3004, 1 reg, 0.01A)
    CMD_IDLE_STOP,         // 11: 停稳后停机 (0x0005)
    CMD_SET_STALL_LIMIT,   // 12: 动态更新驱动器堵转限流百分比 (0x070C)
    CMD_FAULT_RESET,       // 13: 驱动器故障复位 (0x2000 = 0x0007)
    CMD_READ_DRIVER_STATUS // 14: 状态/故障码主动查询 (0x2100, 3 regs: 状态字1, 状态字2, 故障码)
} Motor_Cmd_Type_t;

// 控制消息队列数据结构
typedef struct {
    Motor_Cmd_Type_t cmd_type; // 使用 Motor_Cmd_Type_t 对应的命令值
    uint8_t motor_mask;        // 目标电机选择掩码 (Bit0~Bit3)
    int16_t speed_rpm;         // 目标转速 (RPM) 或通用参数值
} Motor_Ctrl_Msg_t;

// 声明 FreeRTOS 电机控制消息队列句柄
extern QueueHandle_t g_motor_ctrl_queue;

// FreeRTOS 串口通讯与超时任务
void APP_CommTask(void *pvParameters);

// 动态下发驱动器堵转限流百分比更新指令 (向 active_mask 电机写入 0x070C)
void App_Comm_UpdateDriverStallLimit(void);

#endif // __APP_COMM_H
