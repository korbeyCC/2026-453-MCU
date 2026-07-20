#ifndef __APP_COMM_H
#define __APP_COMM_H

#include "app_main.h"
#include "app_control.h" // 导入全局控制状态和结构体

// 核心配置与控制指令字用枚举包装，引入 CMD_READ_HALL 读取指令
typedef enum {
    CMD_NONE = 0,
    CMD_FORWARD,           // 1: 启动正转 (0x0001)
    CMD_REVERSE,           // 2: 启动反转 (0x0002)
    CMD_STOP,              // 3: 刹车停机 (0x0009)
    CMD_INIT_WRITE_ENABLE, // 4: 初始化：0x200E 写使能
    CMD_INIT_RUN_MODE,     // 5: 初始化：0x2006 运行方式选择
    CMD_INIT_SPEED_MODE,   // 6: 初始化：0x2007 速度命令选择
    CMD_INIT_SET_SPEED,    // 7: 初始化：0x2001 目标速度设为 0
    CMD_INIT_START_RUN,    // 8: 初始化：0x2000 启动运转指令
    CMD_READ_HALL          // 9: 周期读取：读取 32 位原始霍尔高度 (0x3013)
} Motor_Cmd_Type_t;

// 控制消息队列数据结构
typedef struct {
    Motor_Cmd_Type_t cmd_type; // 使用 Motor_Cmd_Type_t 对应的命令值
    uint8_t motor_mask;        // 目标电机选择掩码 (Bit0~Bit3)
    int16_t speed_rpm;         // 目标转速 (RPM)
} Motor_Ctrl_Msg_t;

// 声明 FreeRTOS 电机控制消息队列句柄
extern QueueHandle_t g_motor_ctrl_queue;

// FreeRTOS 串口通讯与超时任务
void APP_CommTask(void *pvParameters);

#endif // __APP_COMM_H
