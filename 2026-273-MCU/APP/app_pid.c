#include "app_pid.h"

/**
 * @brief  PID 初始化
 * @param  pid: PID 结构体指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @param  target: 目标设定值
 * @param  out_max: 最大输出限制
 * @param  out_min: 最小输出限制
 * @param  iout_max: 积分饱和限制幅值
 */
void APP_PID_Init(APP_PID_Handle_t *pid, float kp, float ki, float kd, float target, float out_max, float out_min, float iout_max)
{
    if (pid == NULL) return;

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Target = target;
    pid->Out_Max = out_max;
    pid->Out_Min = out_min;
    pid->Iout_Max = iout_max;
    
    // 历史误差与状态清零
    pid->Error[0] = pid->Error[1] = pid->Error[2] = 0.0f;
    pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->Out = out_min; // 默认为最小限幅输出
}

/**
 * @brief  PID 运算 (位置式, 支持正负浮点输出)
 * @param  pid: PID 结构体指针
 * @param  current_val: 当前反馈值
 * @return float: 经限幅处理后的 PID 速度微调量
 */
float APP_PID_Calc(APP_PID_Handle_t *pid, float current_val)
{
    if (pid == NULL) return 0.0f;

    // 计算当前偏差值
    float Error = pid->Target - current_val;
    
    // 滚动更新历史误差
    pid->Error[2] = pid->Error[1];
    pid->Error[1] = pid->Error[0];
    pid->Error[0] = Error;
    
    // 比例项输出
    pid->Pout = pid->Kp * Error;
    
    // 积分项输出，带抗积分饱和限幅
    pid->Iout += pid->Ki * Error;
    if (pid->Iout > pid->Iout_Max)
    {
        pid->Iout = pid->Iout_Max;
    }
    else if (pid->Iout < -pid->Iout_Max)
    {
        pid->Iout = -pid->Iout_Max;
    }
    
    // 微分项输出 (针对当前偏差的一阶差分)
    pid->Dout = pid->Kd * (pid->Error[0] - pid->Error[1]);
    
    // 综合计算三项输出值
    float out_val = pid->Pout + pid->Iout + pid->Dout;
    
    // 对总输出进行绝对限幅保护
    if (out_val > pid->Out_Max)
    {
        out_val = pid->Out_Max;
    }
    else if (out_val < pid->Out_Min)
    {
        out_val = pid->Out_Min;
    }
    
    // 保存并返回
    pid->Out = out_val;
    return pid->Out;
}

/**
 * @brief  PID 状态清除 (在停机/换向等需要清除积分的场景调用)
 * @param  pid: PID 结构体指针
 */
void APP_PID_Clear(APP_PID_Handle_t *pid)
{
    if (pid == NULL) return;
    
    pid->Error[0] = pid->Error[1] = pid->Error[2] = 0.0f;
    pid->Iout = 0.0f; // 清空积分项，防止带入下个周期产生超调
    pid->Out = pid->Out_Min;
}

/**
 * @brief  修改 PID 目标设定值
 * @param  pid: PID 结构体指针
 * @param  new_target: 新的目标值
 */
void APP_PID_SetTarget(APP_PID_Handle_t *pid, float new_target)
{
    if (pid == NULL) return;
    pid->Target = new_target;
}

/**
 * @brief  修改并限制比例系数 Kp
 */
void APP_PID_SetKp(APP_PID_Handle_t *pid, float kp)
{
    if (pid == NULL) return;
    if (kp > 5.0f) kp = 5.0f;
    else if (kp < 0.0f) kp = 0.0f;
    pid->Kp = kp;
}

/**
 * @brief  修改并限制积分系数 Ki
 */
void APP_PID_SetKi(APP_PID_Handle_t *pid, float ki)
{
    if (pid == NULL) return;
    if (ki > 0.2f) ki = 0.2f;
    else if (ki < 0.0f) ki = 0.0f;
    pid->Ki = ki;
}

/**
 * @brief  修改并限制微分系数 Kd
 */
void APP_PID_SetKd(APP_PID_Handle_t *pid, float kd)
{
    if (pid == NULL) return;
    if (kd > 0.1f) kd = 0.1f;
    else if (kd < 0.0f) kd = 0.0f;
    pid->Kd = kd;
}
