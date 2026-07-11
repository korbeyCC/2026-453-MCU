#ifndef __APP_COMM_H
#define __APP_COMM_H

#include "app_main.h"

// 电机初始化状态机步骤
typedef enum {
    MOTOR_INIT_STEP_WRITE_ENABLE = 0, // 0x200E 功能码写使能
    MOTOR_INIT_STEP_RUN_MODE,        // 0x2006 运行命令选通讯方式
    MOTOR_INIT_STEP_SPEED_MODE,      // 0x2007 速度命令选通讯方式
    MOTOR_INIT_STEP_SET_SPEED,       // 0x2001 设定目标转速为 50 RPM
    MOTOR_INIT_STEP_START_RUN,       // 0x2000 启动正转
    MOTOR_INIT_STEP_DONE             // 电机配置完成，进入周期读取阶段
} MotorInitStep;

// 电机状态监控结构体 (便于调试和控制调度)
typedef struct {
    MotorInitStep init_step;     // 电机当前配置状态
    uint32_t hall_value;         // 驱动器返回的 32 位霍尔脉冲计数值
    uint8_t comm_error;          // 通讯错误标记 (0:正常; 1:连续发生超时故障)
    uint8_t retry_cnt;           // 通讯超时重试次数
    uint8_t read_timer;          // 周期性读霍尔轮询时基计数器
} Motor_Status_t;

// 声明全局 4 路电机的状态数据，供 Debug 及控制业务层调用
extern Motor_Status_t g_motor_status[4];

// FreeRTOS 串口通讯与超时任务
void APP_CommTask(void *pvParameters);

#endif // __APP_COMM_H
