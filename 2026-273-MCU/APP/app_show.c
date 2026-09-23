#include "app_Show.h"
#include "app_Data.h"
#include "mid_Key.h"
#include "app_Menu.h"
#include "app_supervisor.h"
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

/**
 * @brief 渲染单项监测数据 (支持 4 轴位置 / 实时电流 或 8 项位置+现场冻结电流)
 * @param item_idx 0~3: 电机 1~4 位置 (mm); 4~7: 电机 1~4 电流 (现场/冻结电流)
 */
static void Show_RenderItemData(uint8_t item_idx, uint8_t *SEG_W, uint8_t *SEG_Flag)
{
    uint8_t mask      = App_Data_GetColumnMotorMask();
    uint8_t max_items = (app_data.show_current_mode == 2) ? 8 : 4;
    item_idx          = item_idx % max_items;

    uint8_t m_idx = (item_idx < 4) ? item_idx : (item_idx - 4);
    if (!(mask & (1 << m_idx))) {
        for (int i = 0; i < 4; i++) {
            if (mask & (1 << i)) {
                m_idx    = (uint8_t)i;
                item_idx = (item_idx < 4) ? m_idx : (m_idx + 4);
                break;
            }
        }
    }

    bool show_curr = (app_data.show_current_mode == 1) ||
                     (app_data.show_current_mode == 2 && item_idx >= 4);

    if (show_curr) {
        static const uint8_t s_motor_chars[4] = {10, 11, 12, 13}; // 'A', 'b', 'c', 'd'
        SEG_W[0] = s_motor_chars[m_idx];

        // 优先获取现场冻结保存的电流；若未发生故障则取实时电流
        uint16_t cur_deciA = 0;
        if (Sys_Mode_IsFaultLocked() || Sys_View_GetFaultCode() != 0) {
            cur_deciA = Sys_View_GetFaultCurrentDeciA(m_idx);
            if (cur_deciA == 0) {
                cur_deciA = Sys_View_GetAxisCurrentDeciA(m_idx);
            }
        } else {
            cur_deciA = Sys_View_GetAxisCurrentDeciA(m_idx);
        }

        uint32_t val_0_1A = (uint32_t)((cur_deciA + 5) / 10);
        if (val_0_1A > 999) val_0_1A = 999;

        uint8_t tens = (uint8_t)((val_0_1A / 100) % 10);
        SEG_W[1]     = (val_0_1A >= 100) ? tens : 19;
        SEG_W[2]     = (uint8_t)((val_0_1A / 10) % 10);
        SEG_W[3]     = (uint8_t)(val_0_1A % 10);

        SEG_Flag[0] = SEG_Flag[1] = SEG_Flag[3] = 0;
        SEG_Flag[2] = 1;
    } else {
        int32_t val_mm = Sys_View_GetAxisTravelMm(m_idx);

        SEG_W[0] = (uint8_t)((val_mm / 1000) % 10);
        SEG_W[1] = (uint8_t)((val_mm / 100) % 10);
        SEG_W[2] = (uint8_t)((val_mm / 10) % 10);
        SEG_W[3] = (uint8_t)(val_mm % 10);

        SEG_Flag[0] = SEG_Flag[1] = SEG_Flag[2] = SEG_Flag[3] = 0;
        SEG_Flag[m_idx] = 1;
    }
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
            if (adjust_hold_ticks > 0 || flicker_cnt < 10) {
                Show_RenderItemData(dim2, SEG_W, SEG_Flag);
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
        // 优先级 2：故障急停报警显示 (在调值/查看期间常显数据，平时与 ErrX 交替显示)
        // ====================================================
        else if (fault_code != 0 || Sys_Mode_IsFaultLocked()) {
            if (adjust_hold_ticks > 0 || flicker_cnt >= 10) {
                // 用户按键浏览期间(adjust_hold_ticks > 0)常显当前项；平时后半周期显示当前项
                Show_RenderItemData(dim2, SEG_W, SEG_Flag);
            } else {
                // 前半周期显示故障代码 (如 Err1, Err3 等)
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

            // 1. 前 500ms（或未调值时交替前半段）显示项编号 `-qX-` 或 `-q10`
            if (adjust_hold_ticks == 0 && flicker_cnt < 10) {
                if (dim2 < 10) {
                    SEG_W[0] = 18;     // -
                    SEG_W[1] = 16;     // q
                    SEG_W[2] = dim2;   // 0 ~ 9
                    SEG_W[3] = 18;     // -
                } else {
                    SEG_W[0] = 18;     // -
                    SEG_W[1] = 16;     // q
                    SEG_W[2] = (uint8_t)(dim2 / 10); // 1
                    SEG_W[3] = (uint8_t)(dim2 % 10); // 0
                }
            }
            // 2. 后 500ms（或调值期间）显示当前项的具体参数数值
            else {
                if (dim2 == 10) {
                    // q10 柱体模式显示："-00-", "-12-", "-13-", "-14-", "-23-", "-24-", "-34-"
                    uint16_t m = APP_Menu_GetEditingColumnMode();
                    SEG_W[0]   = 18; // -
                    if (m == 0) {
                        SEG_W[1] = 0;
                        SEG_W[2] = 0;
                    } else {
                        SEG_W[1] = (uint8_t)((m / 10) % 10);
                        SEG_W[2] = (uint8_t)(m % 10);
                    }
                    SEG_W[3] = 18; // -
                } else if (dim2 == 13) {
                    // q13 堵转判定防抖时间显示：以 0.1s 为单位带小数点，如 600ms 显示 "  0.6", 1200ms 显示 "  1.2"
                    uint16_t val_01s = app_data.stall_detect_time_ms / 100;
                    if (val_01s > 99) val_01s = 99;
                    SEG_W[0] = 19; // 空白
                    SEG_W[1] = 19; // 空白
                    SEG_W[2] = (uint8_t)((val_01s / 10) % 10);
                    SEG_W[3] = (uint8_t)(val_01s % 10);
                    SEG_Flag[2] = 1; // 小数点点亮在第二位(从0开始算第2位，即第3个数码管)
                } else {
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
                        case 11: param_val = app_data.show_current_mode; break;
                        case 12: param_val = app_data.driver_stall_percent; break;
                        default: param_val = 0; break;
                    }

                    if (param_val > 9999) param_val = 9999;

                    if (dim2 == 12) {
                        // q12 显示驱动器百分比：消除千位前导零，如 150 显示为 " 150"
                        SEG_W[0] = (param_val >= 1000) ? (uint8_t)((param_val / 1000) % 10) : 19;
                        SEG_W[1] = (uint8_t)((param_val / 100) % 10);
                        SEG_W[2] = (uint8_t)((param_val / 10) % 10);
                        SEG_W[3] = (uint8_t)(param_val % 10);
                    } else {
                        SEG_W[0] = (uint8_t)((param_val / 1000) % 10);
                        SEG_W[1] = (uint8_t)((param_val / 100) % 10);
                        SEG_W[2] = (uint8_t)((param_val / 10) % 10);
                        SEG_W[3] = (uint8_t)(param_val % 10);
                    }
                }
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
            Show_RenderItemData(dim2, SEG_W, SEG_Flag);
        }

        // 3. 调用中间层驱动转换段码并写入 TM1650 芯片
        MID_SEG_Display(SEG_W, SEG_Flag, 4, SEG_Show_Data);
        MID_TM1650_DisplayWrite(SEG_Show_Data, 4);

        // 每 50ms 刷新一次显示
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(50));
    }
}
