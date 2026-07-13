#ifndef __APP_PID_H
#define __APP_PID_H

#include "main.h"
#include <stdint.h>

/**
 * @brief  PID 控制器结构体 (支持位置式, 已适配支持负向速度调节)
 */
typedef struct {
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数
    float Target;           // 目标设定值 (如高度差目标 0.0)
    float Error[3];         // 历史误差 [当前, 上次, 上上次]
    float Pout;             // 比例输出
    float Iout;             // 积分输出
    float Dout;             // 微分输出
    float Iout_Max;         // 积分限制幅值 (防饱和)
    float Out_Max;          // 输出最大值限制
    float Out_Min;          // 输出最小值限制 (为支持降速微调，可设为负值)
    float Out;              // 最终 PID 输出值
} APP_PID_Handle_t;

// --- API 接口声明 ---
void APP_PID_Init(APP_PID_Handle_t *pid, float kp, float ki, float kd, float target, float out_max, float out_min, float iout_max);
float APP_PID_Calc(APP_PID_Handle_t *pid, float current_val);
void APP_PID_Clear(APP_PID_Handle_t *pid);
void APP_PID_SetTarget(APP_PID_Handle_t *pid, float new_target);
void APP_PID_SetKp(APP_PID_Handle_t *pid, float kp);
void APP_PID_SetKi(APP_PID_Handle_t *pid, float ki);
void APP_PID_SetKd(APP_PID_Handle_t *pid, float kd);

#endif // __APP_PID_H
