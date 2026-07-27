#include "app_Show.h"
#include "app_Data.h"
#include "mid_Key.h"
#include "app_Menu.h"
#include "app_control.h"
#include <math.h>

void APP_ShowTask(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    uint8_t SEG_W[4]    = {19, 19, 19, 19}; // 数码管字模缓冲区
    uint8_t SEG_Flag[4] = {0, 0, 0, 0};     // 小数点缓冲区
    uint8_t SEG_Show_Data[4];               // 输出段码缓冲区

    uint16_t flicker_cnt = 0;               // 交替周期计数器 (50ms 周期)

    while (1) {
        // 维护调值不闪烁计时器
        if (adjust_hold_ticks > 0) {
            adjust_hold_ticks--;
        }

        flicker_cnt++;
        if (flicker_cnt >= 20) {
            flicker_cnt = 0; // 20 * 50ms = 1秒的大交替周期
        }

        // ====================================================
        // 模式一：设置菜单模式 (Set_W = 1 ~ 8)
        // ====================================================
        if (Set_W > 0 && Set_W <= 8) {
            SEG_Flag[0] = SEG_Flag[1] = SEG_Flag[2] = SEG_Flag[3] = 0;

            // 1. 前 500ms（或未调值时交替前半段）显示项编号 `-0X-`
            if (adjust_hold_ticks == 0 && flicker_cnt < 10) {
                SEG_W[0] = 18;     // -
                SEG_W[1] = 0;      // 0
                SEG_W[2] = Set_W;  // 1 ~ 8
                SEG_W[3] = 18;     // -
            }
            // 2. 后 500ms（或调值期间）显示当前项的具体参数数值
            else {
                uint32_t param_val = 0;
                switch (Set_W) {
                    case 1: param_val = app_data.max_travel_range_mm; break;
                    case 2: param_val = app_data.target_speed_mm_min; break;
                    case 3: param_val = app_data.single_tune_speed_rpm; break;
                    case 4: param_val = app_data.max_sync_diff_mm; break;
                    case 5: param_val = app_data.stall_current_threshold; break;
                    case 6: param_val = app_data.lead_mm; break;
                    case 7: param_val = app_data.reduction_ratio; break;
                    case 8: param_val = app_data.single_tune_step_0_1mm; break;
                    default: param_val = 0; break;
                }

                if (param_val > 9999) param_val = 9999;

                SEG_W[0] = (uint8_t)((param_val / 1000) % 10);
                SEG_W[1] = (uint8_t)((param_val / 100) % 10);
                SEG_W[2] = (uint8_t)((param_val / 10) % 10);
                SEG_W[3] = (uint8_t)(param_val % 10);
            }
        }
        // ====================================================
        // 模式二：常规运行模式 (Set_W == 0)
        // ====================================================
        else {
            // 1. 解算当前 show_motor_idx 所指示电机的绝对高度 (单位: mm)
            uint8_t m_idx = show_motor_idx % 4;
            float abs_mm  = 0.0f;
            if (g_sys_context.counts_per_mm > 0.0f) {
                abs_mm = (float)g_sys_context.g_motor_status[m_idx].current_abs_hall / g_sys_context.counts_per_mm;
            }

            int32_t val_mm = (int32_t)roundf(abs_mm);
            if (val_mm < 0) val_mm = 0;
            if (val_mm > 9999) val_mm = 9999;

            // 2. 填充 4 位数码管数值
            SEG_W[0] = (uint8_t)((val_mm / 1000) % 10);
            SEG_W[1] = (uint8_t)((val_mm / 100) % 10);
            SEG_W[2] = (uint8_t)((val_mm / 10) % 10);
            SEG_W[3] = (uint8_t)(val_mm % 10);

            // 3. 小数点指示电机编号 (Motor 0 -> 1 点亮, Motor 1 -> 2 点亮, Motor 2 -> 3 点亮, Motor 3 -> 4 点亮)
            SEG_Flag[0] = SEG_Flag[1] = SEG_Flag[2] = SEG_Flag[3] = 0;
            SEG_Flag[m_idx] = 1;
        }

        // 3. 调用中间层驱动转换段码并写入 TM1650 芯片
        MID_SEG_Display(SEG_W, SEG_Flag, 4, SEG_Show_Data);
        MID_TM1650_DisplayWrite(SEG_Show_Data, 4);

        // 每 50ms 刷新一次显示
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(50));
    }
}
