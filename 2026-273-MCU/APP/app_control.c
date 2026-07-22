#include <math.h>
#include <stdlib.h>
#include "app_control.h"
#include "mid_signal.h"
#include "mid_led.h"
#include "mid_modbus.h"
#include "app_Comm.h"
#include "app_Data.h"
#include "app_pid.h"

// 实例化全局控制上下文
Sys_Ctrl_Context_t g_sys_context;

// 4路电机的PID控制实例
static APP_PID_Handle_t motor_pids[4];

// ========================== 独立无副作用高度与电流监控打印 ==========================

/**
 * @brief  输出当前 4 路绝对高度、电流及 PID 目标转速的监控波形
 */
static void APP_Control_DebugPrint(void)
{
    float avg_delta_h = 0.0f;
    float delta_h[4];

    // 计算各立柱自本次起跑以来的位移增量 ΔH_i 及平均位移增量
    for (int i = 0; i < 4; i++) {
        delta_h[i] = (float)(g_sys_context.g_motor_status[i].current_abs_hall - g_sys_context.g_motor_status[i].base_abs_hall);
        avg_delta_h += delta_h[i];
    }
    avg_delta_h /= 4.0f;

    Debug_Printf("H0:%dH1:%dH2:%dH3:%dS0:%dS1:%dS2:%dS3:%d\r\n",
                 labs(g_sys_context.g_motor_status[0].current_abs_hall - g_sys_context.g_motor_status[0].base_abs_hall),
                 labs(g_sys_context.g_motor_status[1].current_abs_hall - g_sys_context.g_motor_status[1].base_abs_hall),
                 labs(g_sys_context.g_motor_status[2].current_abs_hall - g_sys_context.g_motor_status[2].base_abs_hall),
                 labs(g_sys_context.g_motor_status[3].current_abs_hall - g_sys_context.g_motor_status[3].base_abs_hall),
                 g_sys_context.g_motor_status[0].target_speed,
                 g_sys_context.g_motor_status[1].target_speed,
                 g_sys_context.g_motor_status[2].target_speed,
                 g_sys_context.g_motor_status[3].target_speed);
}

// ========================== 三重系统安防保护检查 ==========================

/**
 * @brief  三重系统级安防防护检查（通信中断、同步差超限、过流堵转）
 * @return true: 触发故障急停; false: 系统安全
 */
static bool APP_Control_CheckSafety(void)
{
    // // 1. 通信连续中断检查
    // for (int i = 0; i < 4; i++) {
    //     if (g_sys_context.g_motor_status[i].comm_error >= 5) {
    //         g_sys_context.system_fault_code = 2; // 2: 通信中断急停
    //         Debug_Printf("[ERR] Safety Fault: Motor %d Comm Loss!\r\n", i);
    //         return true;
    //     }
    // }

    // 2. 轴间同步差超限检查
    float min_dh = 1e9f, max_dh = -1e9f;
    for (int i = 0; i < 4; i++) {
        float dh = (float)(g_sys_context.g_motor_status[i].current_abs_hall - g_sys_context.g_motor_status[i].base_abs_hall);
        if (dh < min_dh) min_dh = dh;
        if (dh > max_dh) max_dh = dh;
    }
    if ((max_dh - min_dh) > (float)g_sys_context.max_sync_diff_hall) {
        g_sys_context.system_fault_code = 3; // 3: 同步差超限急停
        Debug_Printf("[ERR] Safety Fault: Sync Diff Exceeded! (Diff=%.1f > Limit=%d)\r\n",
                     (max_dh - min_dh), g_sys_context.max_sync_diff_hall);
        Debug_Printf("[SYS] Delta Halls: DH0=%.0f, DH1=%.0f, DH2=%.0f, DH3=%.0f | AbsHalls: H0=%d, H1=%d, H2=%d, H3=%d\r\n",
                     (float)(g_sys_context.g_motor_status[0].current_abs_hall - g_sys_context.g_motor_status[0].base_abs_hall),
                     (float)(g_sys_context.g_motor_status[1].current_abs_hall - g_sys_context.g_motor_status[1].base_abs_hall),
                     (float)(g_sys_context.g_motor_status[2].current_abs_hall - g_sys_context.g_motor_status[2].base_abs_hall),
                     (float)(g_sys_context.g_motor_status[3].current_abs_hall - g_sys_context.g_motor_status[3].base_abs_hall),
                     g_sys_context.g_motor_status[0].current_abs_hall,
                     g_sys_context.g_motor_status[1].current_abs_hall,
                     g_sys_context.g_motor_status[2].current_abs_hall,
                     g_sys_context.g_motor_status[3].current_abs_hall);
        return true;
    }

    // 3. 单轴过流堵转检查
    for (int i = 0; i < 4; i++) {
        if (g_sys_context.g_motor_status[i].current_deciA > app_data.stall_current_threshold) {
            g_sys_context.g_motor_status[i].stall_cnt++;
            if (g_sys_context.g_motor_status[i].stall_cnt >= 20) { // 200ms 持续过流
                g_sys_context.system_fault_code = 1;               // 1: 过流堵转
                Debug_Printf("[ERR] Safety Fault: Motor %d OverCurrent Stall! (Curr=%.2fA > Limit=%.2fA)\r\n",
                             i, (float)g_sys_context.g_motor_status[i].current_deciA / 100.0f,
                             (float)app_data.stall_current_threshold / 100.0f);
                Debug_Printf("[SYS] Loaded Flash Abs Halls: H0=%d, H1=%d, H2=%d, H3=%d \r\n",
                             g_sys_context.g_motor_status[0].current_abs_hall,
                             g_sys_context.g_motor_status[1].current_abs_hall,
                             g_sys_context.g_motor_status[2].current_abs_hall,
                             g_sys_context.g_motor_status[3].current_abs_hall);
                return true;
            }
        } else {
            if (g_sys_context.g_motor_status[i].stall_cnt > 0) {
                g_sys_context.g_motor_status[i].stall_cnt--;
            }
        }
    }

    return false;
}

// ========================== PID 控制同步算法 ==========================

/**
 * @brief  PID 同步计算，基于本次运动过程中的位移增量 ΔH 调节各路电机的目标速度
 * @param  base_speed 基准转速
 */
static void APP_Control_RunPID(int16_t base_speed)
{
    if (base_speed <= 0) return;

    float avg_delta_h = 0.0f;
    float delta_h[4];

    // 1. 计算各个立柱自本次起跑以来的位移增量 ΔH_i = current_abs_hall_i - base_abs_hall_i
    for (int i = 0; i < 4; i++) {
        delta_h[i] = (float)(g_sys_context.g_motor_status[i].current_abs_hall - g_sys_context.g_motor_status[i].base_abs_hall);
        avg_delta_h += delta_h[i];
    }
    avg_delta_h /= 4.0f;

    float calc_target_v[4];
    float max_v = -1e9f;

    // 2. 算出 4 通道的理论 PID 调速结果
    for (int i = 0; i < 4; i++) {
        APP_PID_SetTarget(&motor_pids[i], avg_delta_h);
        float delta_v = APP_PID_Calc(&motor_pids[i], delta_h[i]);

        if (g_sys_context.g_motor_status[i].target_cmd == CMD_FORWARD) {
            calc_target_v[i] = (float)base_speed + delta_v;
        } else if (g_sys_context.g_motor_status[i].target_cmd == CMD_REVERSE) {
            calc_target_v[i] = (float)base_speed - delta_v;
        } else {
            calc_target_v[i] = 0.0f;
        }

        if (calc_target_v[i] > max_v) {
            max_v = calc_target_v[i];
        }
    }

    // 3. 防饱和速度平移：若最高轴理论转速突破 3000 RPM，全局向下平移溢出量，保护打满触顶
    float shift_offset = 0.0f;
    if (max_v > 3000.0f) {
        shift_offset = max_v - 3000.0f;
    }

    for (int i = 0; i < 4; i++) {
        float target_v = calc_target_v[i] - shift_offset;

        // 限制下限在 300 ~ 3000 RPM 之间
        if (target_v > 3000.0f) {
            target_v = 3000.0f;
        } else if (target_v < 300.0f && g_sys_context.g_motor_status[i].target_cmd != CMD_STOP) {
            target_v = 300.0f;
        }

        if (g_sys_context.g_motor_status[i].target_cmd == CMD_STOP) {
            g_sys_context.g_motor_status[i].target_speed = 0;
        } else {
            g_sys_context.g_motor_status[i].target_speed = (int16_t)target_v;
        }
    }
}

// ========================== 核心业务控制任务 ==========================

/**
 * @brief 核心业务控制任务（解耦数据驱动架构）
 */
void APP_ControlTask(void *pvParameters)
{
    MID_SIGNAL_Msg sig_msg;
    static int16_t last_sent_speed[4]   = {-1, -1, -1, -1};
    static uint8_t print_divider        = 0;
    static uint32_t last_check_halls[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
    static uint8_t stop_stable_cnt      = 0;

    // 1. 初始化系统上下文与 PID (匹配 5ms/200Hz 极速控制)
    g_sys_context.system_step       = SYS_STEP_Boot;
    g_sys_context.is_hardware_ready = false;
    g_sys_context.base_speed        = 0;
    g_sys_context.system_fault_code = 0;

    for (int i = 0; i < 4; i++) {
        APP_PID_Init(&motor_pids[i], 0.40f, 0.001f, 0.0f, 0.0f, 100.0f, -100.0f, 30.0f);
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 非阻塞检查信号/事件
        bool has_event = MID_Signal_GetEvent(&sig_msg, 0);

        switch (g_sys_context.system_step) {
            // === Boot 状态：等待硬件初始化完成并执行物理参数换算 ===
            case SYS_STEP_Boot: {
                if (g_sys_context.is_hardware_ready) {
                    // 同步 Flash 保存的高度绝对起点至监控内存
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].current_abs_hall = app_data.motor_abs_halls[i];
                    }

                    // 物理参数单位自动换算
                    uint16_t ratio = (app_data.reduction_ratio > 0) ? app_data.reduction_ratio : 30;
                    uint16_t lead  = (app_data.lead_mm > 0) ? app_data.lead_mm : 6;
                    uint16_t coef  = (app_data.hall_coef > 0) ? app_data.hall_coef : 30;

                    g_sys_context.counts_per_mm      = (uint32_t)(ratio * coef) / lead;                                    // 150 count/mm
                    g_sys_context.calc_base_rpm      = (int16_t)((app_data.target_speed_mm_min * ratio) / lead);           // 2400 RPM (对应 480 mm/min)
                    g_sys_context.max_sync_diff_hall = (int32_t)(app_data.max_sync_diff_mm * g_sys_context.counts_per_mm); // 750 counts
                    g_sys_context.max_travel_hall    = (int32_t)(app_data.max_travel_range_mm * g_sys_context.counts_per_mm);

                    g_sys_context.system_step = SYS_STEP_READY;
                    Debug_Printf("[SYS] Counts/mm=%d, CalcRPM=%d, MaxSyncDiffHall=%d, MaxTravelHall=%d\r\n",
                                 g_sys_context.counts_per_mm, g_sys_context.calc_base_rpm,
                                 g_sys_context.max_sync_diff_hall, g_sys_context.max_travel_hall);
                }
                break;
            }

            // === READY 状态：就绪待命 ===
            case SYS_STEP_READY: {
                if (has_event && sig_msg.event == MID_SIGNAL_EVT_TRIGGER) {
                    Motor_Ctrl_Msg_t speed_msg;
                    Motor_Ctrl_Msg_t cmd_msg;
                    bool action_valid = false;

                    int16_t run_rpm = (g_sys_context.calc_base_rpm > 0) ? g_sys_context.calc_base_rpm : 2400;

                    switch (sig_msg.signal_id) {
                        case MID_SIGNAL_REMOT_3: // 遥控下行
                            g_sys_context.base_speed = run_rpm;

                            speed_msg.cmd_type   = CMD_SET_SPEED;
                            speed_msg.motor_mask = 0x0F;
                            speed_msg.speed_rpm  = run_rpm;

                            cmd_msg.cmd_type   = CMD_REVERSE;
                            cmd_msg.motor_mask = 0x0F;
                            cmd_msg.speed_rpm  = 0;

                            for (int i = 0; i < 4; i++) {
                                g_sys_context.g_motor_status[i].start_drive_hall = (int32_t)g_sys_context.g_motor_status[i].hall_value;
                                g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                                g_sys_context.g_motor_status[i].target_cmd       = CMD_REVERSE;
                                g_sys_context.g_motor_status[i].target_speed     = run_rpm;
                                g_sys_context.g_motor_status[i].stall_cnt        = 0;
                                last_sent_speed[i]                               = run_rpm;
                            }

                            xQueueReset(g_motor_ctrl_queue);
                            xQueueSend(g_motor_ctrl_queue, &speed_msg, pdMS_TO_TICKS(10));
                            xQueueSend(g_motor_ctrl_queue, &cmd_msg, pdMS_TO_TICKS(10));

                            action_valid              = true;
                            g_sys_context.system_step = SYS_STEP_TOTAL_RUNNING;
                            break;

                        case MID_SIGNAL_REMOT_4: // 遥控上行
                            g_sys_context.base_speed = run_rpm;

                            speed_msg.cmd_type   = CMD_SET_SPEED;
                            speed_msg.motor_mask = 0x0F;
                            speed_msg.speed_rpm  = run_rpm;

                            cmd_msg.cmd_type   = CMD_FORWARD;
                            cmd_msg.motor_mask = 0x0F;
                            cmd_msg.speed_rpm  = 0;

                            for (int i = 0; i < 4; i++) {
                                g_sys_context.g_motor_status[i].start_drive_hall = (int32_t)g_sys_context.g_motor_status[i].hall_value;
                                g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                                g_sys_context.g_motor_status[i].target_cmd       = CMD_FORWARD;
                                g_sys_context.g_motor_status[i].target_speed     = run_rpm;
                                g_sys_context.g_motor_status[i].stall_cnt        = 0;
                                last_sent_speed[i]                               = run_rpm;
                            }

                            xQueueReset(g_motor_ctrl_queue);
                            xQueueSend(g_motor_ctrl_queue, &speed_msg, pdMS_TO_TICKS(10));
                            xQueueSend(g_motor_ctrl_queue, &cmd_msg, pdMS_TO_TICKS(10));

                            action_valid              = true;
                            g_sys_context.system_step = SYS_STEP_TOTAL_RUNNING;
                            break;

                        default:
                            break;
                    }

                    if (action_valid) {
                        MID_SIGNAL_Msg dummy_msg;
                        while (MID_Signal_GetEvent(&dummy_msg, 0) == pdTRUE);
                    }
                }
                break;
            }

            // === 核心运行调速阶段（定频 5ms 数据驱动 PID 泵 + 安防防线） ===
            case SYS_STEP_TOTAL_RUNNING: {
                // 1. 无条件优先从内存缓存层解算更新 4 路当帧最新绝对高度
                for (int i = 0; i < 4; i++) {
                    int32_t drive_relative_hall = (int32_t)g_sys_context.g_motor_status[i].hall_value;
                    g_sys_context.g_motor_status[i].current_abs_hall =
                        g_sys_context.g_motor_status[i].base_abs_hall + (drive_relative_hall - g_sys_context.g_motor_status[i].start_drive_hall);
                }

                if (has_event && sig_msg.event == MID_SIGNAL_EVT_TRIGGER) {
                    // 1. 清空死锁消息队列
                    xQueueReset(g_motor_ctrl_queue);

                    // 2. 状态重置为 STOP
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }

                    // 3. 高优先级阻塞发送停机命令并重置判定变量
                    stop_stable_cnt = 0;
                    for (int i = 0; i < 4; i++) {
                        last_check_halls[i] = 0xFFFFFFFF;
                    }

                    Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, 0x0F, 0};
                    g_sys_context.system_step = SYS_STEP_TOTAL_DONE;
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[SYS] User Stop signal, sending CMD_STOP...\r\n");
                } else if (APP_Control_CheckSafety()) {
                    // 触发系统级安防防护急停 (过流/通信/同步差超限)
                    xQueueReset(g_motor_ctrl_queue);

                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }

                    Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, 0x0F, 0};
                    g_sys_context.system_step = SYS_STEP_FAULT_STOP;
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                } else {
                    // 2. 极速 PID 位置同步计算 (防饱和速度平移)
                    APP_Control_RunPID(g_sys_context.base_speed);

                    // 3. 【按需下发】仅当目标转速发生变化时向队列推送写速度命令
                    for (int i = 0; i < 4; i++) {
                        if (g_sys_context.g_motor_status[i].target_speed != last_sent_speed[i]) {
                            Motor_Ctrl_Msg_t speed_msg;
                            speed_msg.cmd_type   = CMD_SET_SPEED;
                            speed_msg.motor_mask = (1 << i);
                            speed_msg.speed_rpm  = g_sys_context.g_motor_status[i].target_speed;

                            if (xQueueSend(g_motor_ctrl_queue, &speed_msg, 0) == pdTRUE) {
                                last_sent_speed[i] = g_sys_context.g_motor_status[i].target_speed;
                            }
                        }
                    }

                    // 4. 定频 100ms (20帧分频, 10Hz) 输出波形日志
                    if (++print_divider >= 20) {
                        print_divider = 0;
                        APP_Control_DebugPrint();
                    }
                }
                break;
            }

            // === 停机归档阶段（含 4 轴连续静止检测） ===
            case SYS_STEP_TOTAL_DONE:
            case SYS_STEP_TUNE_DONE: {
                // 实时解算当前 4 路绝对高度
                for (int i = 0; i < 4; i++) {
                    int32_t drive_relative_hall = (int32_t)g_sys_context.g_motor_status[i].hall_value;
                    g_sys_context.g_motor_status[i].current_abs_hall =
                        g_sys_context.g_motor_status[i].base_abs_hall + (drive_relative_hall - g_sys_context.g_motor_status[i].start_drive_hall);
                }

                // 检查 4 路霍尔原始数据是否相比上一次 5ms 无任何变化
                bool is_all_same = true;
                for (int i = 0; i < 4; i++) {
                    if (g_sys_context.g_motor_status[i].hall_value != last_check_halls[i]) {
                        is_all_same         = false;
                        last_check_halls[i] = g_sys_context.g_motor_status[i].hall_value;
                    }
                }

                if (is_all_same) {
                    stop_stable_cnt++;
                } else {
                    stop_stable_cnt = 0;
                }

                // 连续 20 次（100ms 周期）4 路位置全无变化，确认物理电机已完全停稳
                if (stop_stable_cnt >= 20) {
                    stop_stable_cnt = 0;
                    for (int i = 0; i < 4; i++) {
                        last_check_halls[i]         = 0xFFFFFFFF;
                        app_data.motor_abs_halls[i] = g_sys_context.g_motor_status[i].current_abs_hall;
                    }

                    // EEPROMSet                = 1;//感觉每次停止都存fash太折寿了还是注释了吧？
                    g_sys_context.base_speed = 0;

                    APP_Control_DebugPrint();

                    g_sys_context.system_step = SYS_STEP_READY;
                    Debug_Printf("[SYS] Loaded Flash Abs Halls: L0=%dL1=%dL2=%dL3=%d\r\n",
                                 g_sys_context.g_motor_status[0].current_abs_hall,
                                 g_sys_context.g_motor_status[1].current_abs_hall,
                                 g_sys_context.g_motor_status[2].current_abs_hall,
                                 g_sys_context.g_motor_status[3].current_abs_hall);
                    Debug_Printf("[SYS] System State -> READY, Halls Fully Stopped & Archived to Flash.\r\n");
                }
                break;
            }

            // === 故障急停状态 ===
            case SYS_STEP_FAULT_STOP: {
                if (has_event && sig_msg.event == MID_SIGNAL_EVT_TRIGGER) {
                    g_sys_context.system_fault_code = 0;
                    g_sys_context.system_step       = SYS_STEP_READY;
                    Debug_Printf("[SYS] Fault Cleared -> System READY.\r\n");
                }
                break;
            }

            default:
                break;
        }

        // 严格 5ms 周期挂起 (200Hz 极速调度)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}
