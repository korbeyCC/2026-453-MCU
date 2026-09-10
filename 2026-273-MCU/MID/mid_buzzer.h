#ifndef __MID_BUZZER_H
#define __MID_BUZZER_H

#include "mid_main.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 2026-453 运行蜂鸣器 (PD2, BEEP) 工作模式
 */
typedef enum {
    BUZZER_MODE_MUTE = 0,         // 静音 (待机或正常停止)
    BUZZER_MODE_RUNNING,          // 运行提示模式 (每 1000ms 蜂鸣 100ms 提示机械运动)
    BUZZER_MODE_ALARM             // 故障急促报警模式 (100ms鸣 / 100ms停 急促报警)
} buzzer_mode_t;

void MID_Buzzer_Init(void);
void MID_Buzzer_Write(bool on);
void MID_Buzzer_Toggle(void);
void MID_Buzzer_SetMode(buzzer_mode_t mode);
buzzer_mode_t MID_Buzzer_GetMode(void);
void MID_Buzzer_TriggerBeep(uint16_t duration_ms);
void MID_Buzzer_Process(uint16_t interval_ms);

#endif /* __MID_BUZZER_H */
