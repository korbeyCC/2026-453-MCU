#include "app_Menu.h"
#include "mid_Key.h"
#include "app_Data.h"
#include "mid_buzzer.h"
#include "app_supervisor.h"
#include "app_control.h"
#include <stdio.h>
#include <string.h>

// 二维坐标控制变量实例化
uint8_t dim1               = 0; // 0: 实时信息层, 1: 常规设置层, 2: 深层调试纯只读层
uint8_t dim2               = 0; // 维度二具体项 / 电机索引 (0~3)
uint16_t adjust_hold_ticks = 0;

// 数码管提示动画全局变量
char prompt_str[6]    = {0};
uint16_t prompt_ticks = 0;

#if ENABLE_STALL_CURRENT_AUTO_SAVE
static uint8_t save_debounce_cnt = 0; // 0.3s 及时保存倒计时
#endif

/**
 * @brief 设置数码管短闪文本提示动画 (如 "-P1-", "-q2-", "-UP-", "-DW-")
 */
void APP_Menu_SetPrompt(const char *str, uint16_t ticks_50ms)
{
    if (str != NULL) {
        snprintf(prompt_str, sizeof(prompt_str), "%s", str);
        prompt_ticks = ticks_50ms;
    }
}

uint8_t reset_factory_flag        = 0; // 0: 不恢复出厂设置, 1: 恢复出厂设置
static uint8_t s_edit_column_mode = 0; // 菜单第 10 项临时编辑模式值，离开保存时统一生效

uint8_t APP_Menu_GetEditingColumnMode(void)
{
    return s_edit_column_mode;
}

/**
 * @brief 常规应用设置层 (dim1 == 1) 参数加减调节通用辅助函数
 */
static void APP_Menu_AdjustParam(bool is_inc)
{
    adjust_hold_ticks = 30; // 调值期间 1.5s 保持数码管数字稳定不闪烁

    switch (dim2) {
        case 0: // reset_factory_flag (必须设为 7 才能触发恢复出厂设置，防误触)
            if (is_inc) {
                if (reset_factory_flag < 9) reset_factory_flag++;
            } else {
                if (reset_factory_flag > 0) reset_factory_flag--;
            }
            break;

        case 1: // Set_W = 1: max_travel_range_mm (100 ~ 5000mm)
            if (is_inc) {
                if (app_data.max_travel_range_mm <= 4950)
                    app_data.max_travel_range_mm += 50;
                else
                    app_data.max_travel_range_mm = 5000;
            } else {
                if (app_data.max_travel_range_mm >= 150)
                    app_data.max_travel_range_mm -= 50;
                else
                    app_data.max_travel_range_mm = 100;
            }
            break;

        case 2: // Set_W = 2: target_speed_mm_min (100 ~ 2000 mm/min)
            if (is_inc) {
                if (app_data.target_speed_mm_min <= 1950)
                    app_data.target_speed_mm_min += 50;
                else
                    app_data.target_speed_mm_min = 2000;
            } else {
                if (app_data.target_speed_mm_min >= 150)
                    app_data.target_speed_mm_min -= 50;
                else
                    app_data.target_speed_mm_min = 100;
            }
            break;

        case 3: // Set_W = 3: motor_dir_invert (0: 默认正向, 1: 极性反转)
            if (is_inc) {
                if (app_data.motor_dir_invert < 1)
                    app_data.motor_dir_invert++;
                else
                    app_data.motor_dir_invert = 1;
            } else {
                if (app_data.motor_dir_invert > 0)
                    app_data.motor_dir_invert--;
                else
                    app_data.motor_dir_invert = 0;
            }
            break;

        case 4: // Set_W = 4: max_sync_diff_mm (1 ~ 50 mm)
            if (is_inc) {
                if (app_data.max_sync_diff_mm < 50) app_data.max_sync_diff_mm++;
            } else {
                if (app_data.max_sync_diff_mm > 1) app_data.max_sync_diff_mm--;
            }
            break;

        case 5: // Set_W = 5: stall_current_threshold (限制在 [STALL_CURRENT_THRESHOLD_MIN, STALL_CURRENT_THRESHOLD_MAX] 之内)
            if (is_inc) {
                if (app_data.stall_current_threshold + 10 <= STALL_CURRENT_THRESHOLD_MAX)
                    app_data.stall_current_threshold += 10;
                else
                    app_data.stall_current_threshold = STALL_CURRENT_THRESHOLD_MAX;
            } else {
                if (app_data.stall_current_threshold >= STALL_CURRENT_THRESHOLD_MIN + 10)
                    app_data.stall_current_threshold -= 10;
                else
                    app_data.stall_current_threshold = STALL_CURRENT_THRESHOLD_MIN;
            }
#if ENABLE_STALL_CURRENT_AUTO_SAVE
            save_debounce_cnt = 6; // 仅在启用宏时触发 0.3s 及时保存
#endif
            break;

        case 6: // Set_W = 6: lead_mm (1 ~ 50 mm)
            if (is_inc) {
                if (app_data.lead_mm < 50) app_data.lead_mm++;
            } else {
                if (app_data.lead_mm > 1) app_data.lead_mm--;
            }
            break;

        case 7: // Set_W = 7: reduction_ratio (1 ~ 200)
            if (is_inc) {
                if (app_data.reduction_ratio < 200) app_data.reduction_ratio++;
            } else {
                if (app_data.reduction_ratio > 1) app_data.reduction_ratio--;
            }
            break;

        case 8: // Set_W = 8: single_tune_step_mm (1 ~ 200 mm，默认 1mm)
            if (is_inc) {
                if (app_data.single_tune_step_mm < 200) app_data.single_tune_step_mm++;
            } else {
                if (app_data.single_tune_step_mm > 1) app_data.single_tune_step_mm--;
            }
            break;

        case 9: // Set_W = 9: rebound_travel_mm (10 ~ 5000 mm，默认 1000mm)
            if (is_inc) {
                if (app_data.rebound_travel_mm <= 4990)
                    app_data.rebound_travel_mm += 10;
                else
                    app_data.rebound_travel_mm = 5000;
            } else {
                if (app_data.rebound_travel_mm >= 20)
                    app_data.rebound_travel_mm -= 10;
                else
                    app_data.rebound_travel_mm = 10;
            }
            break;

        case 10: { // Set_W = 10: column_mode (0: 四柱 "00", 12, 13, 14, 23, 24, 34)
            // 1. 已锁定且为四柱模式：只能看，不能调！
            if (app_data.column_mode_locked == 1 && app_data.column_mode == 0) {
                s_edit_column_mode = 0;
                break;
            }

            // 2. 模式切换安全门禁：必须所有柱子都低于或等于安装起点 (即处于底部机械零位) 才允许切换！
            if (!APP_Control_IsAllColumnsAtBottom()) {
                APP_Menu_SetPrompt("-Err-", 20); // 闪烁显示 "-Err-" 1.0秒
                MID_Buzzer_TriggerBeep(100);     // 蜂鸣器长鸣 100ms 提示操作被拒
                Debug_Printf("[SYS] Column Mode Switch Denied! All columns must be at bottom origin (current_abs <= min_mount).\r\n");
                break;
            }

            static const uint16_t s_modes[7] = {0, 12, 13, 14, 23, 24, 34};
            int8_t cur_idx                   = 0;
            for (int i = 0; i < 7; i++) {
                if (s_edit_column_mode == s_modes[i]) {
                    cur_idx = (int8_t)i;
                    break;
                }
            }

            if (app_data.column_mode_locked == 0) {
                // 未锁定态 (出厂默认或重置出厂后)：允许在 00 与 6 种双柱模式之间任意自由切换
                if (is_inc) {
                    cur_idx = (cur_idx + 1) % 7;
                } else {
                    cur_idx = (cur_idx + 6) % 7;
                }
                s_edit_column_mode = s_modes[cur_idx];
            } else {
                // 已锁定态：允许在 6 种双柱组合之间切换，但绝不能切换回 00！
                if (cur_idx < 1) cur_idx = 1;
                if (is_inc) {
                    cur_idx = 1 + ((cur_idx - 1 + 1) % 6);
                } else {
                    cur_idx = 1 + ((cur_idx - 1 + 5) % 6);
                }
                s_edit_column_mode = s_modes[cur_idx];
            }
            break;
        }

        default:
            break;
    }
}

void APP_MenuTask(void *pvParameters)
{
    MID_KEY_SingleKeyMsg msg;
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

#if SYS_ROUTER_USE_FREERTOS
    // 自注册任务句柄至路由器引擎，启用 Carrier B (Task Notification 极速唤醒)
    Sys_Router_RegisterMenuTask(xTaskGetCurrentTaskHandle());
#endif

    while (1) {
        // 50ms 递减提示动画倒计时
        if (prompt_ticks > 0) {
            prompt_ticks--;
        }

#if ENABLE_STALL_CURRENT_AUTO_SAVE
        // 50ms 定频处理 0.3s 及时保存定时器
        if (save_debounce_cnt > 0) {
            save_debounce_cnt--;
            if (save_debounce_cnt == 0) {
                APP_Data_Storage(); // 0.3s 无新按键，及时固化存 Flash
                Sys_Notify_ParamsUpdated();
                Debug_Printf("[SYS] Stall Current Threshold Auto Saved to Flash (StallCurrent=%d, 0.01A).\r\n",
                             app_data.stall_current_threshold);
            }
        }
#endif

        // 载体 B：通过任务通知无损接收菜单按键 (0 队列 RAM 开销)
        bool has_key = false;
        uint8_t k_id = 0, k_evt = 0, k_cnt = 0;
#if SYS_ROUTER_USE_FREERTOS
        if (Sys_Router_WaitMenuKey(&k_id, &k_evt, &k_cnt, 0)) {
            msg.key_id = (MID_Key_ID)k_id;
            msg.event  = (MID_KeyEventType)k_evt;
            has_key    = true;
        } else
#endif
            if (MID_Key_GetSingleEvent(&msg, 0) == pdTRUE) {
            has_key = true;
        }

        if (has_key) {
            // 连发事件 (MID_KEY_EVT_Long_REP) 时静音，避免长按调参时蜂鸣器持续吵闹
            if (msg.event != MID_KEY_EVT_Long_REP) {
                MID_Buzzer_TriggerBeep(40); // 453 按键有效触发提示音 40ms
            }

            // ====================================================
            // 维度长按切换：长按 K6 (设置键) 切换一维 dim1 (0 -> 1 -> 2 -> 0)
            // ====================================================
            if (msg.key_id == MID_KEY_ID_K6 && msg.event == MID_KEY_EVT_LONG) {
                uint8_t next_dim1 = (dim1 + 1) % 3;

                // 门禁检查：若试图进入设置模式 (dim1 == 1)，必须确保电机未处于运动或调平状态
                if (next_dim1 == 1 && !Sys_Mode_CanEnterMenu()) {
                    APP_Menu_SetPrompt("-Err-", 20); // 提示错误，拒绝在运动中调参
                    Debug_Printf("[SYS] Enter Menu Blocked: Motor is Currently Running or Aligning!\r\n");
                    continue;
                }

                // 如果离开设置模式，存盘 Flash
                if (dim1 == 1) {
                    if (reset_factory_flag == 7) {
                        reset_factory_flag = 0;
                        APP_Data_ResetDefault();   // (1, 0) 设为 7 触发恢复出厂设置
                        Sys_Notify_FactoryReset(); // 全面重置系统上下文状态与 4 轴运行位置
                        Debug_Printf("[SYS] Factory Reset Executed via Menu (1, 0 = 7)!\r\n");
                    } else {
                        reset_factory_flag          = 0;
                        app_data.column_mode        = s_edit_column_mode; // 真正生效并固化
                        app_data.column_mode_locked = 1;                  // 一旦保存设置，立即进入单向锁定态
                        APP_Data_Storage();
                        Sys_Notify_ParamsUpdated();
                        Debug_Printf("[SYS] Menu Level 1 Params Saved to Flash (ColumnMode=%d, Locked=1).\r\n", app_data.column_mode);
                    }
                    Sys_Mode_Set(SYS_MODE_STANDBY); // 退出设置模式，恢复待机态
                }

                dim1              = next_dim1;
                dim2              = 0; // 切入新维度时均从第 0 项 (0) 开始！
                adjust_hold_ticks = 0;

                if (dim1 == 1) {
                    s_edit_column_mode = app_data.column_mode; // 进入设置菜单时同步初始化临时编辑模式值
                    Sys_Mode_Set(SYS_MODE_MENU_CONFIG);        // 切入设置模式，独占按键并安全封锁电机
                } else if (dim1 == 2) {
                    Sys_Mode_Set(SYS_MODE_DEBUG_CALIB); // 切入深度调试层
                } else {
                    Sys_Mode_Set(SYS_MODE_STANDBY);
                }

                char buf[10];
                snprintf(buf, sizeof(buf), "-P%d-", dim1);
                APP_Menu_SetPrompt(buf, 40); // 切换维度提示 "-P0-", "-P1-", "-P2-" (保持 2.0s)
                Debug_Printf("[SYS] Menu Switched to Dimension 1 = %d (%s)\r\n",
                             dim1, (dim1 == 0) ? "REALTIME" : (dim1 == 1) ? "APP_SETTING"
                                                                          : "DEBUG_READONLY");
            }
            // ====================================================
            // 维度 0：主界面 & 实时监测层 (dim1 == 0)
            // ====================================================
            else if (dim1 == 0) {
                // 1. 短按 K1 或 K6：轮播切换当前模式下的有效使能轴实时读数
                if ((msg.key_id == MID_KEY_ID_K1 || msg.key_id == MID_KEY_ID_K6) && msg.event == MID_KEY_EVT_LEASS) {
                    uint8_t mask     = App_Data_GetColumnMotorMask();
                    uint8_t next_idx = dim2;
                    for (int step = 1; step <= 4; step++) {
                        uint8_t candidate = (dim2 + step) % 4;
                        if (mask & (1 << candidate)) {
                            next_idx = candidate;
                            break;
                        }
                    }
                    dim2 = next_idx;
                    Debug_Printf("[SYS] Display Switched to Motor %d Absolute Hall/Travel.\r\n", dim2);
                }
                // 2. 短按 K5：切换微调方向 (0:正转/上升, 1:反转/下降)，并闪烁提示 "-UP-" / "-dn-"
                else if (msg.key_id == MID_KEY_ID_K5 && msg.event == MID_KEY_EVT_LEASS) {
                    uint8_t new_dir = (Sys_View_GetTuneDir() == 0) ? 1 : 0;
                    Sys_View_SetTuneDir(new_dir);
                    if (new_dir == 0) {
                        APP_Menu_SetPrompt("-UP-", 20); // 闪烁显示 "-UP-" 1.0秒
                        Debug_Printf("[SYS] Single Tune Direction Switched to: FORWARD (UP)\r\n");
                    } else {
                        APP_Menu_SetPrompt("-dn-", 20); // 闪烁显示 "-dn-" 1.0秒
                        Debug_Printf("[SYS] Single Tune Direction Switched to: REVERSE (DOWN)\r\n");
                    }
                }
                // (注意：K1 ~ K4 微调动作键已在待机态直接由中枢路由给 APP_ControlTask 自治驱动，无需菜单介入)
            }
            // ====================================================
            // 维度 1：常规应用设置层 (dim1 == 1, dim2 为 0~10, 其中 (1,0) 为恢复出厂开关)
            // ====================================================
            else if (dim1 == 1) {
                // A. 短按 K6：前进到下一项 (dim2++)。在最后一项 (dim2 == 10) 按 K6 时保存 Flash 并退出至 dim1 = 0
                if (msg.key_id == MID_KEY_ID_K6 && msg.event == MID_KEY_EVT_LEASS) {
                    if (dim2 < 10) {
                        dim2++;
                        adjust_hold_ticks = 0;

                        char buf[10];
                        snprintf(buf, sizeof(buf), "-q%d-", dim2);
                        APP_Menu_SetPrompt(buf, 20); // 切换项目显示 "-q0-", "-q1-"... 1.0s
                        Debug_Printf("[SYS] Setting Next Item: dim2 = %d\r\n", dim2);
                    } else {
                        // 最后一项 (10) 按 K6：检查 (1,0) 是否调至 7 (q0 == 7 触发恢复出厂)
                        if (reset_factory_flag == 7) {
                            reset_factory_flag = 0;
                            APP_Data_ResetDefault();   // 恢复全部出厂默认参数并存盘 Flash (解除锁定)
                            Sys_Notify_FactoryReset(); // 全面重置系统上下文状态与 4 轴运行位置
                            dim1              = 0;
                            dim2              = 0;
                            adjust_hold_ticks = 0;
                            Sys_Mode_Set(SYS_MODE_STANDBY);
                            APP_Menu_SetPrompt("-rSt-", 30); // 闪烁显示 "-rSt-" (Reset) 1.5s
                            Debug_Printf("[SYS] Factory Reset Executed via Menu (1, 0 = 7)! Restored Default Factory Settings.\r\n");
                        } else {
                            reset_factory_flag          = 0;
                            app_data.column_mode        = s_edit_column_mode; // 真正生效并固化
                            app_data.column_mode_locked = 1;                  // 一旦保存设置，立即进入单向锁定态
                            APP_Data_Storage();
                            Sys_Notify_ParamsUpdated();
                            dim1              = 0;
                            dim2              = 0;
                            adjust_hold_ticks = 0;
                            Sys_Mode_Set(SYS_MODE_STANDBY);
                            APP_Menu_SetPrompt("-P0-", 20);
                            Debug_Printf("[SYS] Menu Setting Complete & Saved to Flash (ColumnMode=%d, Locked=1)! Exit to dim1 = 0.\r\n", app_data.column_mode);
                        }
                    }
                }
                // B. 短按 K5：后退到上一项 (dim2--)。在第一项 (dim2 == 0) 按 K5 时不保存直接退出至 dim1 = 0
                else if (msg.key_id == MID_KEY_ID_K5 && msg.event == MID_KEY_EVT_LEASS) {
                    if (dim2 > 0) {
                        dim2--;
                        adjust_hold_ticks = 0;

                        char buf[10];
                        snprintf(buf, sizeof(buf), "-q%d-", dim2);
                        APP_Menu_SetPrompt(buf, 20);
                        Debug_Printf("[SYS] Setting Prev Item: dim2 = %d\r\n", dim2);
                    } else {
                        // 第一项 (1,0) 按 K5：不保存退出
                        reset_factory_flag = 0;
                        s_edit_column_mode = app_data.column_mode; // 放弃编辑，恢复原值
                        dim1               = 0;
                        dim2               = 0;
                        adjust_hold_ticks  = 0;
                        Sys_Mode_Set(SYS_MODE_STANDBY);
                        APP_Menu_SetPrompt("-P0-", 20);
                        Debug_Printf("[SYS] Menu Setting Cancelled (No Save). Exit to dim1 = 0.\r\n");
                    }
                }
                // C. K1 (+) / K2 (-) 参数调节 (短按松手单步响应 + 长按快速连发)
                else if ((msg.key_id == MID_KEY_ID_K1 || msg.key_id == MID_KEY_ID_K2) &&
                         (msg.event == MID_KEY_EVT_LEASS || msg.event == MID_KEY_EVT_LONG || msg.event == MID_KEY_EVT_Long_REP)) {
                    APP_Menu_AdjustParam(msg.key_id == MID_KEY_ID_K1); // 通过 K1/K2 按键 ID 判断是增加还是减少
                }
            }
            // ====================================================
            // 维度 2：深层调试与专家层 (dim1 == 2, 纯只读 Read-Only, 查看 4 轴基准零点 mm)
            // ====================================================
            else if (dim1 == 2) {
                // A. 短按 K6：前进到下一项只读查看 (dim2: 0~3 对应 4 轴零点 mm)
                if (msg.key_id == MID_KEY_ID_K6 && msg.event == MID_KEY_EVT_LEASS) {
                    if (dim2 < 3) {
                        dim2++;
                    } else {
                        dim1 = 0;
                        dim2 = 0;
                        APP_Menu_SetPrompt("-P0-", 20);
                        Debug_Printf("[SYS] Read-Only View Completed. Exit to dim1 = 0.\r\n");
                        continue;
                    }
                    char buf[10];
                    snprintf(buf, sizeof(buf), "-q%d-", dim2);
                    APP_Menu_SetPrompt(buf, 20);
                    Debug_Printf("[SYS] Debug Read-Only View Next Item: dim2 = %d\r\n", dim2);
                }
                // B. 短按 K5：后退到上一项或退出
                else if (msg.key_id == MID_KEY_ID_K5 && msg.event == MID_KEY_EVT_LEASS) {
                    if (dim2 > 0) {
                        dim2--;
                        char buf[10];
                        snprintf(buf, sizeof(buf), "-q%d-", dim2);
                        APP_Menu_SetPrompt(buf, 20);
                        Debug_Printf("[SYS] Debug Read-Only View Prev Item: dim2 = %d\r\n", dim2);
                    } else {
                        dim1 = 0;
                        dim2 = 0;
                        APP_Menu_SetPrompt("-P0-", 20);
                        Debug_Printf("[SYS] Exit Debug Read-Only View to dim1 = 0.\r\n");
                    }
                }
                // C. K1 / K2 按键在 dim1 == 2 (只读层) 中完全忽略！防止误改深层零点或底层数据
                else if (msg.key_id == MID_KEY_ID_K1 || msg.key_id == MID_KEY_ID_K2) {
                    Debug_Printf("[SYS] Read-Only Mode: K1/K2 Param Adjustment Ignored.\r\n");
                }
            }
        }

        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(50));
    }
}
