#ifndef __APP_MENU_H
#define __APP_MENU_H

#include "app_main.h"

// 设置菜单状态机全局变量
extern uint8_t Set_W;            // 0: 常规显示模式; 1~8: 菜单设置模式
extern uint16_t adjust_hold_ticks; // 调值不闪烁倒计时 (单位: 50ms 周期)
extern uint8_t show_motor_idx;   // Set_W == 0 时当前数码管显示的电机索引 (0~3)

void APP_MenuTask(void *pvParameters);

#endif
