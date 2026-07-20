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

// ========================== 独立无副作用高度波形打印 ==========================

/**
 * @brief  输出当前 4 路绝对高度和 PID 目标转速的监控波形
 */
static void APP_Control_DebugPrint(void)
{
    float avg_travel = 0.0f;
    float travel_rel[4];

    // 计算各立柱当前相对行程和平均相对位移
    for (int i = 0; i < 4; i++) {
        travel_rel[i] = (float)(g_sys_context.g_motor_status[i].current_abs_hall - app_data.min_mount_halls[i]);
        avg_travel += travel_rel[i];
    }
    avg_travel /= 4.0f;

    Debug_Printf("H0:%d,H1:%d,H2:%d,H3:%d,AvgTravel:%.1f,V0:%d,V1:%d,V2:%d,V3:%d\r\n",
                 g_sys_context.g_motor_status[0].current_abs_hall,
                 g_sys_context.g_motor_status[1].current_abs_hall,
                 g_sys_context.g_motor_status[2].current_abs_hall,
                 g_sys_context.g_motor_status[3].current_abs_hall,
                 avg_travel,
                 g_sys_context.g_motor_status[0].target_speed,
                 g_sys_context.g_motor_status[1].target_speed,
                 g_sys_context.g_motor_status[2].target_speed,
                 g_sys_context.g_motor_status[3].target_speed);
}

// ========================== PID 控制同步算法 ==========================

/**
 * @brief  PID 同步计算，基于平均相对行程调节各路电机的目标速度
 * @param  base_speed 基准转速
 */
static void APP_Control_RunPID(int16_t base_speed)
{
    if (base_speed <= 0) return;

    float avg_travel = 0.0f;
    float travel_rel[4];

    // 1. 计算各个立柱的当前相对行程位移
    for (int i = 0; i < 4; i++) {
        travel_rel[i] = (float)(g_sys_context.g_motor_status[i].current_abs_hall - app_data.min_mount_halls[i]);
        avg_travel += travel_rel[i];
    }
    avg_travel /= 4.0f;

    // 2. 对每个通道单独计算 PID 同步位置修正
    for (int i = 0; i < 4; i++) {
        APP_PID_SetTarget(&motor_pids[i], avg_travel);
        float delta_v  = APP_PID_Calc(&motor_pids[i], travel_rel[i]);
        float target_v = 0.0f;

        if (g_sys_context.g_motor_status[i].target_cmd == CMD_FORWARD) {
            target_v = (float)base_speed + delta_v;
        } else if (g_sys_context.g_motor_status[i].target_cmd == CMD_REVERSE) {
            target_v = (float)base_speed - delta_v;
        } else {
            target_v = 0.0f;
        }

        // 限制调速范围在 30 ~ 200 RPM
        if (target_v > 200.0f)
            target_v = 200.0f;
        else if (target_v < 30.0f && g_sys_context.g_motor_status[i].target_cmd != CMD_STOP)
            target_v = 30.0f;

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

    // 1. 初始化系统上下文与 PID
    g_sys_context.system_step       = SYS_STEP_Boot;
    g_sys_context.is_hardware_ready = false;
    g_sys_context.base_speed        = 0;

    for (int i = 0; i < 4; i++) {
        APP_PID_Init(&motor_pids[i], 0.5f, 0.01f, 0.0f, 0.0f, 50.0f, -50.0f, 20.0f);
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 非阻塞检查信号/事件
        bool has_event = MID_Signal_GetEvent(&sig_msg, 0);

        switch (g_sys_context.system_step) {
            // === Boot 状态：等待硬件初始化完成 ===
            case SYS_STEP_Boot: {
                if (g_sys_context.is_hardware_ready) {
                    // 同步 Flash 保存的高度绝对起点至监控内存
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].current_abs_hall = app_data.motor_abs_halls[i];
                    }

                    g_sys_context.system_step = SYS_STEP_READY;
                    Debug_Printf("[SYS] System State -> READY (Loaded Halls: H0=%d H1=%d H2=%d H3=%d)\r\n",
                                 app_data.motor_abs_halls[0], app_data.motor_abs_halls[1],
                                 app_data.motor_abs_halls[2], app_data.motor_abs_halls[3]);
                }
                break;
            }

            // === READY 状态：就绪待命 ===
            case SYS_STEP_READY: {
                if (has_event && sig_msg.event == MID_SIGNAL_EVT_TRIGGER) {
                    Motor_Ctrl_Msg_t speed_msg;
                    Motor_Ctrl_Msg_t cmd_msg;
                    bool action_valid = false;

                    switch (sig_msg.signal_id) {
                        case MID_SIGNAL_REMOT_3: // 遥控下行
                            g_sys_context.base_speed = 100;

                            speed_msg.cmd_type   = CMD_SET_SPEED;
                            speed_msg.motor_mask = 0x0F;
                            speed_msg.speed_rpm  = 100;

                            cmd_msg.cmd_type   = CMD_REVERSE;
                            cmd_msg.motor_mask = 0x0F;
                            cmd_msg.speed_rpm  = 0;

                            for (int i = 0; i < 4; i++) {
                                g_sys_context.g_motor_status[i].start_drive_hall = (int32_t)g_sys_context.g_motor_status[i].hall_value;
                                g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                                g_sys_context.g_motor_status[i].target_cmd       = CMD_REVERSE;
                                g_sys_context.g_motor_status[i].target_speed     = 100;
                                last_sent_speed[i]                               = 100;
                            }

                            xQueueReset(g_motor_ctrl_queue);
                            xQueueSend(g_motor_ctrl_queue, &speed_msg, pdMS_TO_TICKS(10));
                            xQueueSend(g_motor_ctrl_queue, &cmd_msg, pdMS_TO_TICKS(10));

                            action_valid              = true;
                            g_sys_context.system_step = SYS_STEP_TOTAL_RUNNING;
                            break;

                        case MID_SIGNAL_REMOT_4: // 遥控上行
                            g_sys_context.base_speed = 100;

                            speed_msg.cmd_type   = CMD_SET_SPEED;
                            speed_msg.motor_mask = 0x0F;
                            speed_msg.speed_rpm  = 100;

                            cmd_msg.cmd_type   = CMD_FORWARD;
                            cmd_msg.motor_mask = 0x0F;
                            cmd_msg.speed_rpm  = 0;

                            for (int i = 0; i < 4; i++) {
                                g_sys_context.g_motor_status[i].start_drive_hall = (int32_t)g_sys_context.g_motor_status[i].hall_value;
                                g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                                g_sys_context.g_motor_status[i].target_cmd       = CMD_FORWARD;
                                g_sys_context.g_motor_status[i].target_speed     = 100;
                                last_sent_speed[i]                               = 100;
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

            // === 核心运行调速阶段（定频 10ms 数据驱动 PID 泵） ===
            case SYS_STEP_TOTAL_RUNNING: {
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
                } else {
                    // 1. 从内存缓存层解算 4 路绝对高度
                    for (int i = 0; i < 4; i++) {
                        int32_t drive_relative_hall = (int32_t)g_sys_context.g_motor_status[i].hall_value;
                        g_sys_context.g_motor_status[i].current_abs_hall =
                            g_sys_context.g_motor_status[i].base_abs_hall + (drive_relative_hall - g_sys_context.g_motor_status[i].start_drive_hall);
                    }

                    // 2. 执行定频 PID 位置同步计算
                    APP_Control_RunPID(g_sys_context.base_speed);

                    // 3. 定频 100ms(10Hz) 输出波形日志
                    if (++print_divider >= 10) {
                        print_divider = 0;
                        APP_Control_DebugPrint();
                    }

                    // 4. 【按需下发】仅当目标转速发生变化时向队列推送写速度命令
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

                // 检查 4 路霍尔原始数据是否相比上一次 10ms 无任何变化
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

                // 连续 10 次（100ms 周期）4 路位置全无变化，确认物理电机已完全停稳
                if (stop_stable_cnt >= 10) {
                    stop_stable_cnt = 0;
                    for (int i = 0; i < 4; i++) {
                        last_check_halls[i]         = 0xFFFFFFFF;
                        app_data.motor_abs_halls[i] = g_sys_context.g_motor_status[i].current_abs_hall;
                    }

                    // EEPROMSet                = 1;//感觉每次停止都存fash太折寿了还是注释了吧？
                    g_sys_context.base_speed = 0;

                    APP_Control_DebugPrint();

                    g_sys_context.system_step = SYS_STEP_READY;
                }
                break;
            }

            default:
                break;
        }

        // 严格 10ms 周期挂起
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}
