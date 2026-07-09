#include "app_Menu.h"
#include "mid_Key.h"
#include "app_Data.h"

uint8_t Set_W = 0;
uint16_t adjust_hold_ticks = 0;

void APP_MenuTask(void *pvParameters)
{
    MID_KEY_SingleKeyMsg msg;
    
    while (1)
    {
        // 阻塞接收按键事件
        if (MID_Key_GetSingleEvent(&msg, portMAX_DELAY) == pdTRUE)
        {
            // 1. K6 (设置键) 短按松开 -> 切换并增加 Set_W
            if (msg.key_id == MID_KEY_ID_K6 && msg.event == MID_KEY_EVT_LEASS)
            {
                if (Set_W == 0)
                {
                    Set_W = 1;
                }
                else
                {
                    Set_W = (Set_W + 1) % 6; // 设置状态 0 ~ 5 循环
                    if (Set_W == 0)
                    {
                        EEPROMSet = 1; // 退出设置状态时触发 Flash 保存
                    }
                }
            }
            
            // 2. 在设置状态下，处理 K2(加) 和 K3(减) 等调节动作的预留接口
            if (Set_W > 0)
            {
                // 只要在设置状态下触发按键，就维持 1.5 秒数码管不闪烁
                adjust_hold_ticks = 30;
                
                if (msg.key_id == MID_KEY_ID_K2)
                {
                    if (msg.event == MID_KEY_EVT_PRESS || msg.event == MID_KEY_EVT_Long_REP)
                    {
                        // 预留 K2 参数加逻辑，例如调整限位高度
                    }
                }
                else if (msg.key_id == MID_KEY_ID_K3)
                {
                    if (msg.event == MID_KEY_EVT_PRESS || msg.event == MID_KEY_EVT_Long_REP)
                    {
                        // 预留 K3 参数减逻辑
                    }
                }
            }
        }
    }
}
