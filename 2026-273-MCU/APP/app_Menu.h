#ifndef __APP_MENU_H
#define __APP_MENU_H

#include "app_main.h"

// 设置菜单状态机全局变量
extern uint8_t Set_W;
extern uint16_t adjust_hold_ticks;

void APP_MenuTask(void *pvParameters);

#endif
