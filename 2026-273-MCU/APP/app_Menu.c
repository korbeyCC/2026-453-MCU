#include "app_Menu.h"
#include "mid_Key.h"
#include "app_Data.h"
#include "app_control.h"

uint8_t Set_W              = 0;
uint16_t adjust_hold_ticks = 0;
uint8_t show_motor_idx     = 0; // 默认显示第一个电机 (Motor 0)

static uint8_t save_debounce_cnt = 0; // 0.3s (300ms) 及时保存倒计时

/**
 * @brief 参数加减调节通用辅助函数
 */
static void APP_Menu_AdjustParam(bool is_inc)
{
    adjust_hold_ticks = 30; // 调值期间 1.5s 保持数码管数字稳定不闪烁

    switch (Set_W) {
        case 1: // max_travel_range_mm (100 ~ 5000mm)
            if (is_inc) {
                if (app_data.max_travel_range_mm <= 4950) app_data.max_travel_range_mm += 50;
                else app_data.max_travel_range_mm = 5000;
            } else {
                if (app_data.max_travel_range_mm >= 150) app_data.max_travel_range_mm -= 50;
                else app_data.max_travel_range_mm = 100;
            }
            break;

        case 2: // target_speed_mm_min (100 ~ 2000 mm/min)
            if (is_inc) {
                if (app_data.target_speed_mm_min <= 1950) app_data.target_speed_mm_min += 50;
                else app_data.target_speed_mm_min = 2000;
            } else {
                if (app_data.target_speed_mm_min >= 150) app_data.target_speed_mm_min -= 50;
                else app_data.target_speed_mm_min = 100;
            }
            break;

        case 3: // single_tune_speed_rpm (10 ~ 1500 RPM)
            if (is_inc) {
                if (app_data.single_tune_speed_rpm <= 1490) app_data.single_tune_speed_rpm += 10;
                else app_data.single_tune_speed_rpm = 1500;
            } else {
                if (app_data.single_tune_speed_rpm >= 20) app_data.single_tune_speed_rpm -= 10;
                else app_data.single_tune_speed_rpm = 10;
            }
            break;

        case 4: // max_sync_diff_mm (1 ~ 50 mm)
            if (is_inc) {
                if (app_data.max_sync_diff_mm < 50) app_data.max_sync_diff_mm++;
            } else {
                if (app_data.max_sync_diff_mm > 1) app_data.max_sync_diff_mm--;
            }
            break;

        case 5: // stall_current_threshold (50 ~ 2000 = 0.50A ~ 20.00A)
            if (is_inc) {
                if (app_data.stall_current_threshold <= 1990) app_data.stall_current_threshold += 10;
                else app_data.stall_current_threshold = 2000;
            } else {
                if (app_data.stall_current_threshold >= 60) app_data.stall_current_threshold -= 10;
                else app_data.stall_current_threshold = 50;
            }
            // 专用于 Set_W == 5 (堵转电流)：调值后触发 0.3s (6*50ms) 及时保存定时器
            save_debounce_cnt = 6;
            break;

        case 6: // lead_mm (1 ~ 50 mm)
            if (is_inc) {
                if (app_data.lead_mm < 50) app_data.lead_mm++;
            } else {
                if (app_data.lead_mm > 1) app_data.lead_mm--;
            }
            break;

        case 7: // reduction_ratio (1 ~ 100)
            if (is_inc) {
                if (app_data.reduction_ratio < 100) app_data.reduction_ratio++;
            } else {
                if (app_data.reduction_ratio > 1) app_data.reduction_ratio--;
            }
            break;

        case 8: // single_tune_step_0_1mm (1 ~ 200 = 0.1mm ~ 20.0mm)
            if (is_inc) {
                if (app_data.single_tune_step_0_1mm < 200) app_data.single_tune_step_0_1mm++;
            } else {
                if (app_data.single_tune_step_0_1mm > 1) app_data.single_tune_step_0_1mm--;
            }
            break;

        default:
            break;
    }
}

void APP_MenuTask(void *pvParameters)
{
    MID_KEY_SingleKeyMsg msg;
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    while (1) {
        // 50ms 定频处理 0.3s 及时保存定时器
        if (save_debounce_cnt > 0) {
            save_debounce_cnt--;
            if (save_debounce_cnt == 0) {
                APP_Data_Storage(); // 0.3s 无新按键，及时固化存 Flash
                APP_Control_UpdateParamsFromAppData();
                Debug_Printf("[SYS] Set_W=5 Current Threshold Auto Saved to Flash (StallCurrent=%d, 0.01A).\r\n",
                             app_data.stall_current_threshold);
            }
        }

        // 独占阻塞/非阻塞检查全局唯一按键队列
        if (MID_Key_GetSingleEvent(&msg, 0) == pdTRUE) {
            // 如果系统正处于单轴微调动作中，按下任意按键均取消微调
            if (g_sys_context.system_step == SYS_STEP_SINGLE_TUNE) {
                if (msg.event == MID_KEY_EVT_LEASS || msg.event == MID_KEY_EVT_PRESS) {
                    APP_Control_CancelSingleTune();
                    Debug_Printf("[SYS] Single Tune Interrupted by Key Press.\r\n");
                }
            }
            // ====================================================
            // 模式一：常规运行模式 (Set_W == 0)
            // ====================================================
            else if (Set_W == 0) {
                // 1. 短按 K6：依次切换数码管显示的电机编号 (Motor 0 -> 1 -> 2 -> 3 -> 0)
                if (msg.key_id == MID_KEY_ID_K6 && msg.event == MID_KEY_EVT_LEASS) {
                    show_motor_idx = (show_motor_idx + 1) % 4;
                    Debug_Printf("[SYS] Display Switched to Motor %d Absolute Hall/Travel.\r\n", show_motor_idx);
                }
                // 2. 长按 K6：进入设置模式 Set_W = 1
                else if (msg.key_id == MID_KEY_ID_K6 && msg.event == MID_KEY_EVT_LONG) {
                    Set_W             = 1;
                    adjust_hold_ticks = 0;
                    Debug_Printf("[SYS] Enter Menu Setting Mode: Set_W = 1 (MaxTravelRange).\r\n");
                }
                // 3. 短按 K5：切换微调方向 (0:正转/上升, 1:反转/下降)
                else if (msg.key_id == MID_KEY_ID_K5 && msg.event == MID_KEY_EVT_LEASS) {
                    g_sys_context.single_tune_dir = (g_sys_context.single_tune_dir == 0) ? 1 : 0;
                    Debug_Printf("[SYS] Single Tune Direction Switched to: %s\r\n",
                                 (g_sys_context.single_tune_dir == 0) ? "FORWARD (UP)" : "REVERSE (DOWN)");
                }
                // 4. 在 SYS_STEP_READY 状态下，短按 K1 ~ K4：发起对应通道单轴微调
                else if (msg.key_id <= MID_KEY_ID_K4 && msg.event == MID_KEY_EVT_LEASS) {
                    uint8_t m_idx = (uint8_t)(msg.key_id - MID_KEY_ID_K1);
                    APP_Control_StartSingleTune(m_idx);
                }
            }
            // ====================================================
            // 模式二：菜单设置模式 (Set_W = 1 ~ 8)
            // ====================================================
            else {
                // A. 短按 K6：跳到下一项 (Set_W++)。在最后一项 (Set_W == 8) 时按 K6，保存 Flash 并退出设置模式
                if (msg.key_id == MID_KEY_ID_K6 && msg.event == MID_KEY_EVT_LEASS) {
                    if (Set_W < 8) {
                        Set_W++;
                        adjust_hold_ticks = 0;
                        Debug_Printf("[SYS] Setting Next Item: Set_W = %d\r\n", Set_W);
                    } else {
                        // 最后一项 Set_W == 8 后保存并退出
                        APP_Data_Storage();
                        APP_Control_UpdateParamsFromAppData();
                        Set_W             = 0;
                        adjust_hold_ticks = 0;
                        Debug_Printf("[SYS] Menu Setting Complete & Saved to Flash! Exit to Set_W = 0.\r\n");
                    }
                }
                // B. 短按 K5：跳到上一项 (Set_W--)。在第一项 (Set_W == 1) 时按 K5，不保存直接退出
                else if (msg.key_id == MID_KEY_ID_K5 && msg.event == MID_KEY_EVT_LEASS) {
                    if (Set_W > 1) {
                        Set_W--;
                        adjust_hold_ticks = 0;
                        Debug_Printf("[SYS] Setting Prev Item: Set_W = %d\r\n", Set_W);
                    } else {
                        // 第一项 Set_W == 1 按 K5：放弃保存直接退出
                        Set_W             = 0;
                        adjust_hold_ticks = 0;
                        Debug_Printf("[SYS] Menu Setting Cancelled (No Save). Exit to Set_W = 0.\r\n");
                    }
                }
                // C. 短按/长按 K1 (+)：增加参数值
                else if (msg.key_id == MID_KEY_ID_K1 &&
                         (msg.event == MID_KEY_EVT_PRESS || msg.event == MID_KEY_EVT_LEASS || msg.event == MID_KEY_EVT_Long_REP)) {
                    if (msg.event != MID_KEY_EVT_LEASS) { // 避免短按释放重复加
                        APP_Menu_AdjustParam(true);
                    }
                }
                // D. 短按/长按 K2 (-)：减少参数值
                else if (msg.key_id == MID_KEY_ID_K2 &&
                         (msg.event == MID_KEY_EVT_PRESS || msg.event == MID_KEY_EVT_LEASS || msg.event == MID_KEY_EVT_Long_REP)) {
                    if (msg.event != MID_KEY_EVT_LEASS) { // 避免短按释放重复减
                        APP_Menu_AdjustParam(false);
                    }
                }
            }
        }

        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(50));
    }
}
