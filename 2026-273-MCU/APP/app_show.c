#include "app_Show.h"
#include "app_Data.h"
#include "mid_Key.h"
#include "app_Menu.h"
#include "app_control.h"
#include <math.h>

/**
 * @brief 字符转换为字模表索引
 */
static uint8_t CharToSegIndex(char c)
{
    if (c >= '0' && c <= '9') return (c - '0');
    if (c == '-') return 18;
    if (c == 'E' || c == 'e') return 14;
    if (c == 'r' || c == 'R') return 28; // r
    if (c == 'P' || c == 'p') return 17;
    if (c == 'U' || c == 'u') return 21;
    if (c == 'D' || c == 'd') return 13;
    if (c == 'N' || c == 'n') return 27; // n
    if (c == 'W' || c == 'w') return 26; // W
    if (c == 'q' || c == 'Q') return 16; // q
    return 19; // 空白
}

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
        // 最高优先级 1：故障急停报警显示 (显示 ErrX, 如 Err1:堵转, Err2:通信中断, Err3:同步差超限)
        // ====================================================
        if (g_sys_context.system_step == SYS_STEP_FAULT_STOP) {
            SEG_Flag[0] = SEG_Flag[1] = SEG_Flag[2] = SEG_Flag[3] = 0;
            SEG_W[0] = 14; // E
            SEG_W[1] = 28; // r
            SEG_W[2] = 28; // r
            uint8_t fault = g_sys_context.system_fault_code;
            if (fault >= 1 && fault <= 9) {
                SEG_W[3] = fault;
            } else {
                SEG_W[3] = 18; // -
            }
        }
        // ====================================================
        // 优先级 2：提示动画 (当 prompt_ticks > 0 时，显示如 "-P1-", "-q2-", "-UP-", "-DW-")
        // ====================================================
        else if (prompt_ticks > 0) {
            SEG_Flag[0] = SEG_Flag[1] = SEG_Flag[2] = SEG_Flag[3] = 0;
            size_t len = strlen(prompt_str);
            for (int i = 0; i < 4; i++) {
                if (i < len) {
                    SEG_W[i] = CharToSegIndex(prompt_str[i]);
                } else {
                    SEG_W[i] = 19;
                }
            }
        }
        // ====================================================
        // 模式一：常规应用设置层 (dim1 == 1)
        // ====================================================
        else if (dim1 == 1) {
            SEG_Flag[0] = SEG_Flag[1] = SEG_Flag[2] = SEG_Flag[3] = 0;

            // 1. 前 500ms（或未调值时交替前半段）显示项编号 `-qX-`
            if (adjust_hold_ticks == 0 && flicker_cnt < 10) {
                SEG_W[0] = 18;     // -
                SEG_W[1] = 16;     // q
                SEG_W[2] = dim2;   // 1 ~ 6
                SEG_W[3] = 18;     // -
            }
            // 2. 后 500ms（或调值期间）显示当前项的具体参数数值
            else {
                uint32_t param_val = 0;
                switch (dim2) {
                    case 0: param_val = reset_factory_flag; break; // (1, 0) 项显示恢复出厂开关
                    case 1: param_val = app_data.max_travel_range_mm; break;
                    case 2: param_val = app_data.target_speed_mm_min; break;
                    case 3: param_val = app_data.motor_dir_invert; break;
                    case 4: param_val = app_data.max_sync_diff_mm; break;
                    case 5: param_val = app_data.stall_current_threshold; break;
                    case 6: param_val = app_data.lead_mm; break;
                    case 7: param_val = app_data.reduction_ratio; break;
                    case 8: param_val = app_data.single_tune_step_mm; break;
                    case 9: param_val = app_data.rebound_travel_mm; break;
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
        // 模式二：深层调试与专家层 (dim1 == 2, 纯只读 Read-Only, 查看 4 轴基准安装零点 mm)
        // ====================================================
        else if (dim1 == 2) {
            SEG_Flag[0] = SEG_Flag[1] = SEG_Flag[2] = SEG_Flag[3] = 0;

            // 1. 前 500ms 交替显示项目编号 `-qX-`
            if (flicker_cnt < 10) {
                SEG_W[0] = 18;        // -
                SEG_W[1] = 16;        // q
                SEG_W[2] = dim2 % 4;  // 0 ~ 3
                SEG_W[3] = 18;        // -
            }
            // 2. 后 500ms 显示该电机基准安装零点的毫米 (mm) 数值
            else {
                uint8_t m_idx = dim2 % 4;
                float mount_mm = 0.0f;
                if (g_sys_context.counts_per_mm > 0.0f) {
                    mount_mm = (float)app_data.min_mount_halls[m_idx] / g_sys_context.counts_per_mm;
                }

                int32_t val_mm = (int32_t)roundf(mount_mm);
                if (val_mm < 0) val_mm = 0;
                if (val_mm > 9999) val_mm = 9999;

                SEG_W[0] = (uint8_t)((val_mm / 1000) % 10);
                SEG_W[1] = (uint8_t)((val_mm / 100) % 10);
                SEG_W[2] = (uint8_t)((val_mm / 10) % 10);
                SEG_W[3] = (uint8_t)(val_mm % 10);

                SEG_Flag[m_idx] = 1; // 点亮小数点指示电机编号
            }
        }
        // ====================================================
        // 模式三：主界面 & 实时监测层 (dim1 == 0)
        // ====================================================
        else {
            // 1. 解算当前 dim2 所指示电机的绝对高度 (单位: mm)
            uint8_t m_idx = dim2 % 4;
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
