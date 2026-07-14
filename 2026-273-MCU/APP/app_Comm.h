#ifndef __APP_COMM_H
#define __APP_COMM_H

#include "app_main.h"

// 控制命令定义
#define CMD_FORWARD 1  // 启动正转 (0x0001)
#define CMD_REVERSE 2  // 启动反转 (0x0002)
#define CMD_STOP    3  // 停机 (0x0005)

// 控制消息队列数据结构
typedef struct {
    uint8_t cmd_type;  // CMD_FORWARD, CMD_REVERSE, CMD_STOP
    int16_t speed_rpm; // 目标转速 (RPM)
} Motor_Ctrl_Msg_t;

// 电机初始化及运行状态机步骤
typedef enum {
    MOTOR_INIT_STEP_WRITE_ENABLE = 0, // 0x200E 功能码写使能
    MOTOR_INIT_STEP_RUN_MODE,        // 0x2006 运行命令选通讯方式
    MOTOR_INIT_STEP_SPEED_MODE,      // 0x2007 速度命令选通讯方式
    MOTOR_INIT_STEP_SET_SPEED,       // 0x2001 设定目标转速为 50 RPM
    MOTOR_INIT_STEP_START_RUN,       // 0x2000 启动正转
    MOTOR_INIT_STEP_DONE             // 电机配置完成，进入正常运行控制及周期霍尔读取阶段
} MotorInitStep;

// 电机运行与状态监控结构体 (有符号化升级，支持起点差值与溢出校正)
typedef struct {
    MotorInitStep init_step;     // 电机当前配置/运行状态步骤
    int32_t base_abs_hall;       // 本次运行起步前的绝对高度基准 (有符号 int32_t)
    int32_t start_drive_hall;    // 本次起步时驱动器的原始霍尔读数起点 (有符号 int32_t)
    int32_t current_abs_hall;    // 实时解算出的绝对高度 (有符号 int32_t)
    uint32_t hall_value;         // 驱动器返回的 32 位原始无符号霍尔脉冲计数值
    uint8_t comm_error;          // 通讯错误标记 (0:正常; 1:通讯超时故障)
    uint8_t retry_cnt;           // 通讯超时重试次数
    
    // 控制与状态对比机制 (实现异步非阻塞精准控制)
    uint8_t target_cmd;          // 目标运行指令 (CMD_FORWARD/REVERSE/STOP)
    int16_t target_speed;        // 目标转速
    
    uint8_t current_cmd;         // 驱动器当前真实的运行指令
    int16_t current_speed;       // 驱动器当前真实的转速配置
} Motor_Status_t;

// 声明全局 4 路电机监控变量
extern Motor_Status_t g_motor_status[4];

// 声明 FreeRTOS 电机控制消息队列句柄
extern QueueHandle_t g_motor_ctrl_queue;

// FreeRTOS 串口通讯与超时任务
void APP_CommTask(void *pvParameters);

#endif // __APP_COMM_H
