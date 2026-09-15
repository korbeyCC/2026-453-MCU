#ifndef __APP_MENU_H
#define __APP_MENU_H

#include "app_main.h"

// 堵转电流调节 0.3s 自动存 Flash 功能宏开关 (0: 暂时屏蔽/关闭, 1: 开启)
#define ENABLE_STALL_CURRENT_AUTO_SAVE 0

// 二维坐标菜单全局控制变量
extern uint8_t dim1;               // 维度一：0-实时信息监测层 (dim2为电机索引 0~3), 1-常规应用设置层, 2-深层调试只读层
extern uint8_t dim2;               // 维度二：具体项目编号 / 电机索引
extern uint8_t reset_factory_flag; // 恢复出厂设置开关标记 (0: 否, 1: 恢复)
extern uint16_t adjust_hold_ticks; // 调值不闪烁倒计时 (单位: 50ms 周期)

// 数码管提示动画全局变量
extern char prompt_str[6];    // 数码管提示文本 (如 "-P1-", "-q2-", "-UP-", "-DW-")
extern uint16_t prompt_ticks; // 提示文本保持显示倒计时 (单位: 50ms 周期)

void APP_Menu_SetPrompt(const char *str, uint16_t ticks_50ms);
void APP_MenuTask(void *pvParameters);
uint8_t APP_Menu_GetEditingColumnMode(void);

#endif // __APP_MENU_H
