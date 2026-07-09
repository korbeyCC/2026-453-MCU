#include "app_Show.h"
#include "app_Data.h"
#include "mid_Key.h"
#include "app_Menu.h"
#include "mid_signal.h"

extern volatile uint8_t tm1650_raw_key;

void APP_ShowTask(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    uint8_t SEG_W[4] = {19, 19, 19, 19}; // 数码管字模缓冲区
    uint8_t SEG_Flag[4] = {0, 0, 0, 0};  // 小数点缓冲区
    uint8_t SEG_Show_Data[4];            // 输出段码缓冲区
    
    uint16_t flicker_cnt = 0;            // 闪烁交替周期计数器 (50ms 周期)
    
    while (1)
    {
        // 维护调值不闪烁计时器
        if (adjust_hold_ticks > 0)
        {
            adjust_hold_ticks--;
        }
        
        flicker_cnt++;
        if (flicker_cnt >= 20)
        {
            flicker_cnt = 0; // 20 * 50ms = 1秒的大交替周期
        }
        
        // ==========================================
        // 模式一：设置菜单模式 (Set_W > 0)
        // ==========================================
        if (Set_W > 0)
        {
            SEG_W[0] = 17; // P
            SEG_W[1] = 18; // -
            
            // 设定参数值闪烁逻辑（调值时强制不闪烁）
            if (adjust_hold_ticks > 0)
            {
                SEG_W[2] = Set_W;
            }
            else if (flicker_cnt < 10) // 约500ms亮
            {
                SEG_W[2] = Set_W;
            }
            else // 约500ms灭
            {
                SEG_W[2] = 19; // 灭
            }
            
            SEG_W[3] = 19; // 灭
            SEG_Flag[0] = SEG_Flag[1] = SEG_Flag[2] = SEG_Flag[3] = 0;
        }
        // ==========================================
        // 模式二：普通模式 (Set_W == 0) 显示按键顺序
        // ==========================================
        else
        {
            uint8_t val = tm1650_raw_key;
            
            // 1. 第一位：显示板载按键状态 (K1~K6, 空闲显示 -)
            if (val == 84)       SEG_W[0] = 1; // K1
            else if (val == 92)  SEG_W[0] = 2; // K2
            else if (val == 100) SEG_W[0] = 3; // K3
            else if (val == 108) SEG_W[0] = 4; // K4
            else if (val == 76)  SEG_W[0] = 5; // K5
            else if (val == 68)  SEG_W[0] = 6; // K6
            else                 SEG_W[0] = 18; // - (空闲)
            
            // 2. 第二位：显示遥控信号状态 (R1~R5, 空闲显示 -)
            if (MID_Signal_GetState(MID_SIGNAL_REMOT_1))      SEG_W[1] = 1;
            else if (MID_Signal_GetState(MID_SIGNAL_REMOT_2)) SEG_W[1] = 2;
            else if (MID_Signal_GetState(MID_SIGNAL_REMOT_3)) SEG_W[1] = 3;
            else if (MID_Signal_GetState(MID_SIGNAL_REMOT_4)) SEG_W[1] = 4;
            else if (MID_Signal_GetState(MID_SIGNAL_REMOT_5)) SEG_W[1] = 5;
            else                                              SEG_W[1] = 18; // -
            
            // 3. 第三位：显示外接下按键状态 (触发显示 d, 空闲显示 -)
            if (MID_Signal_GetState(MID_SIGNAL_BUTON_DW))     SEG_W[2] = 13; // d
            else                                              SEG_W[2] = 18; // -
            
            // 4. 第四位：显示外接上按键状态 (触发显示 U, 空闲显示 -)
            if (MID_Signal_GetState(MID_SIGNAL_BUTON_UP))     SEG_W[3] = 21; // U
            else                                              SEG_W[3] = 18; // -
            
            SEG_Flag[0] = SEG_Flag[1] = SEG_Flag[2] = SEG_Flag[3] = 0;
        }
        
        // 3. 调用中间层转换段码并写入 TM1650
        MID_SEG_Display(SEG_W, SEG_Flag, 4, SEG_Show_Data);
        MID_TM1650_DisplayWrite(SEG_Show_Data, 4);
        
        // 每 50ms 刷新一次显示
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(50));
    }
}
