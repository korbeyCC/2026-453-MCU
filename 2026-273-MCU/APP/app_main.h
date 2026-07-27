#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#include "main.h"
#include <string.h>
#include "cmsis_os.h"
#include "mid_main.h"

#include "app_Show.h"
#include "app_Data.h"
#include "app_Menu.h"
#include "app_control.h"
#include "app_Comm.h"

// 调试串口全局格式化打印与二进制下发输出
void Debug_Printf(const char *format, ...);
void Debug_SendData(const uint8_t *data, uint16_t len);

void APP_Init(void);

#endif
