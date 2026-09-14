#include "app_Show.h"
#include "app_Data.h"
#include "mid_Key.h"
#include "app_Menu.h"
#include "mid_supervisor.h"
#include "mid_run_led.h"
#include "mid_buzzer.h"
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

#define HW_TEST_LED_BEEP_TOGGLE_2S 0 // 1: 硬件测试模式(LED与蜂鸣器两秒同步翻转), 0: 恢复正常业务模式

    uint16_t flicker_cnt = 0;               // 交替周期计数器 (50ms 周期)

#if HW_TEST_LED_BEEP_TOGGLE_2S
    // 上电初始同步亮起并鸣叫
    MID_RunLED_Write(true);
    MID_Buzzer_Write(true);
#endif

    while (1) {
#if HW_TEST_LED_BEEP_TOGGLE_2S
        // 【硬件测试模式】LED 与蜂鸣器每 2 秒同步翻转一次 (2s 亮+鸣 / 2s 灭+静)
        static uint16_t s_hw_test_timer = 0;
        static bool s_hw_test_state = true;
        s_hw_test_timer += 50;
        if (s_hw_test_timer >= 2000) {
            s_hw_test_timer = 0;
            s_hw_test_state = !s_hw_test_state;
            MID_RunLED_Write(s_hw_test_state);
            MID_Buzzer_Write(s_hw_test_state);
        }
#else
        // 2026-453 状态指示灯 (PA12) 与运行蜂鸣器 (PD2) 联动控制 (接入中立视图模型)
        Sys_Indicator_State_t ind = Sys_View_GetIndicatorState();
        if (ind == SYS_IND_FAULT) {
            MID_RunLED_SetMode(RUN_LED_MODE_BLINK_FAULT);
            MID_Buzzer_SetMode(BUZZER_MODE_ALARM);
        } else if (ind == SYS_IND_RUNNING) {
            MID_RunLED_SetMode(RUN_LED_MODE_BLINK_RUN);
            MID_Buzzer_SetMode(BUZZER_MODE_RUNNING);
        } else {
            MID_RunLED_SetMode(RUN_LED_MODE_STEADY_ON);
            MID_Buzzer_SetMode(BUZZER_MODE_MUTE);
        }

        // 50ms 步进驱动指示灯与蜂鸣器节拍
        MID_RunLED_Process(50);
        MID_Buzzer_Process(50);
#endif

        // 维护调值不闪烁计时器
        if (adjust_hold_ticks > 0) {
            adjust_hold_ticks--;
        }

        flicker_cnt++;
        if (flicker_cnt >= 20) {
            flicker_cnt = 0; // 20 * 50ms = 1秒的大交替周期
        }

        // 获取系统当前有效故障码 (0 为无故障)
        uint8_t fault_code = Sys_View_GetFaultCode();

        // ====================================================
        // 最高优先级 1：反弹阶段交替显示 (500ms 实时位置 <-> 500ms 错误码)
        // ====================================================
        if (Sys_View_IsRebounding()) {
            if (flicker_cnt < 10) {
                // 前 500ms 显示当前立柱实时高度 (mm)
                uint8_t m_idx = dim2 % 4;
                int32_t val_mm = Sys_View_GetAxisTravelMm(m_idx);

                SEG_W[0] = (uint8_t)((val_mm / 1000) % 10);
                SEG_W[1] = (uint8_t)((val_mm / 100) % 10);
                SEG_W[2] = (uint8_t)((val_mm / 10) % 10);
                SEG_W[3] = (uint8_t)(val_mm % 10);

                SEG_Flag[0] = SEG_Flag[1] = SEG_Flag[2] = SEG_Flag[3] = 0;
                SEG_Flag[m_idx] = 1; // 点亮当前电机小数点
            } else {
                // 后 500ms 显示故障代码 (如 Err4)
                SEG_Flag[0] = SEG_Flag[1] = SEG_Flag[2] = SEG_Flag[3] = 0;
                SEG_W[0] = 14; // E
                SEG_W[1] = 28; // r
                SEG_W[2] = 28; // r
                if (fault_code >= 1 && fault_code <= 9) {
                    SEG_W[3] = fault_code;
                } else {
                    SEG_W[3] = 18; // -
                }
            }
        }
        // ====================================================
        // 优先级 2：故障急停报警显示 (显示 ErrX, 如 Err1:堵转, Err2:通信中断, Err3:同步差超限, Err4:防夹反弹)
        // ====================================================
        else if (fault_code != 0 || Sys_Mode_IsFaultLocked()) {
            SEG_Flag[0] = SEG_Flag[1] = SEG_Flag[2] = SEG_Flag[3] = 0;
            SEG_W[0] = 14; // E
            SEG_W[1] = 28; // r
            SEG_W[2] = 28; // r
            if (fault_code >= 1 && fault_code <= 9) {
                SEG_W[3] = fault_code;
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
            // 2. 后 500ms 显示该电机基准安装零点的毫米 (mm) 数值 (接入中立视图模型)
            else {
                uint8_t m_idx = dim2 % 4;
                int32_t val_mm = Sys_View_GetAxisMountMm(m_idx);

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
            // 1. 解算当前 dim2 所指示电机的绝对高度 (单位: mm, 接入中立视图模型)
            uint8_t m_idx = dim2 % 4;
            int32_t val_mm = Sys_View_GetAxisTravelMm(m_idx);

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
