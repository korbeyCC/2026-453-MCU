#include "mid_brake.h"
#include "main.h"

/**
 * @brief 机械抱闸接口初始化 (PA15, 默认抱紧锁死)
 */
void MID_Brake_Init(void)
{
    // 上电默认高电平: 关断继电器 J3，P7 接口断电，抱闸失电机械咬紧自锁
    MID_Brake_Lock();
}

/**
 * @brief 松开抱闸 (运行时调用)
 * @note PA15 拉低 -> 光耦 U_Q3 导通 -> Q4 饱和导通 -> 继电器 J3 吸合 -> P7 端子输出 DC 24V -> 电磁铁吸合松开抱闸
 */
void MID_Brake_Release(void)
{
    HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 抱紧抱闸 (停机/待机/急停时调用)
 * @note PA15 置高 -> 光耦 U_Q3 截止 -> Q4 截止 -> 继电器 J3 释放断开 -> P7 端子输出 0V -> 电磁铁失电强力咬紧
 */
void MID_Brake_Lock(void)
{
    HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, GPIO_PIN_SET);
}

/**
 * @brief 查询抱闸当前释放状态
 * @return true: 抱闸已通电松开, false: 抱闸已失电抱死
 */
bool MID_Brake_IsReleased(void)
{
    return (HAL_GPIO_ReadPin(BRAKE_GPIO_Port, BRAKE_Pin) == GPIO_PIN_RESET);
}
