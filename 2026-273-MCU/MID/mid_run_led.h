#ifndef __MID_RUN_LED_H
#define __MID_RUN_LED_H

#include "mid_main.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 2026-453 工作指示灯 (PA12, RUN_LED) 运行模式
 */
typedef enum {
    RUN_LED_MODE_OFF = 0,         // 关闭
    RUN_LED_MODE_STEADY_ON,      // 常亮 (待机就绪状态)
    RUN_LED_MODE_BLINK_RUN,      // 运行慢闪 (1Hz: 250ms亮 / 250ms灭)
    RUN_LED_MODE_BLINK_FAULT     // 故障急闪 (5Hz: 100ms亮 / 100ms灭)
} run_led_mode_t;

void MID_RunLED_Init(void);
void MID_RunLED_Write(bool on);
void MID_RunLED_Toggle(void);
void MID_RunLED_SetMode(run_led_mode_t mode);
run_led_mode_t MID_RunLED_GetMode(void);
void MID_RunLED_Process(uint16_t interval_ms);

#endif /* __MID_RUN_LED_H */
