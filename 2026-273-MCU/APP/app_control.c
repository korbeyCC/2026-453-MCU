#include <math.h>
#include <stdlib.h>
#include "app_control.h"
#include "mid_signal.h"
#include "mid_modbus.h"
#include "app_Comm.h"
#include "app_Data.h"
#include "app_pid.h"
#include "mid_Key.h"
#include "app_Menu.h"
#include "mid_brake.h"
#include "mid_buzzer.h"
#include "app_supervisor.h"

// 实例化全局控制上下文
Sys_Ctrl_Context_t g_sys_context;

// 4路电机的PID控制实例
static APP_PID_Handle_t motor_pids[4];

// ========================== 独立无副作用高度与电流监控打印 ==========================

#pragma pack(1)
typedef struct {
    uint8_t header[2];           // 0xAA, 0x55 (2B)
    uint8_t system_step;         // 系统流程状态机 (0~8) (1B)
    uint8_t system_fault_code;   // 故障代码 (0:正常, 1:过流堵转, 2:通信中断, 3:同步差超限) (1B)
    uint16_t max_travel_diff;    // 当帧最大伸出行程极差/伸出差 (counts) (2B)
    uint16_t max_sync_diff_hall; // 同步差保护上限阀值 (counts) (2B)
    int32_t abs_hall[4];         // 4轴物理伸出长度 (16B) -> 【用于示波器伸出波形绘制】
    int32_t current_abs_hall[4]; // 4轴原始累计绝对霍尔 (16B) -> 【集成新增，用于真实绝对位置分析】
    int32_t delta_h[4];          // 4轴单次运动相对位移增量 ΔH_i (16B) -> 【仅在日志中显示】
    int16_t target_speed[4];     // 4轴目标转速 RPM (0~3000) (8B)
    uint16_t current_deciA[4];   // 4轴实时电流 (0.01A) (8B)
    uint8_t comm_error[4];       // 4轴通信错误计数 (4B)
    uint8_t tail[2];             // 0x0D, 0x0A ('\r\n') (2B)
} Debug_Binary_Frame_t;          // 共 78 字节
#pragma pack()

/**
 * @brief  输出当前 4 路伸出长度、绝对霍尔、电流及 PID 目标转速的二进制高密度数据帧 (78 字节)
 */
static void APP_Control_DebugPrint(void)
{
    Debug_Binary_Frame_t frame;
    frame.header[0]          = 0xAA;
    frame.header[1]          = 0x55;
    frame.system_step        = (uint8_t)g_sys_context.system_step;
    frame.system_fault_code  = g_sys_context.system_fault_code;
    frame.max_travel_diff    = (uint16_t)(g_sys_context.max_travel_diff > 65535.0f ? 65535.0f : g_sys_context.max_travel_diff);
    frame.max_sync_diff_hall = (uint16_t)(g_sys_context.max_sync_diff_hall > 65535 ? 65535 : g_sys_context.max_sync_diff_hall);

    for (int i = 0; i < 4; i++) {
        frame.abs_hall[i]         = (int32_t)g_sys_context.travel_rel[i];
        frame.current_abs_hall[i] = g_sys_context.g_motor_status[i].current_abs_hall;
        frame.delta_h[i]          = (int32_t)g_sys_context.delta_h[i];
        frame.target_speed[i]     = g_sys_context.g_motor_status[i].target_speed;
        frame.current_deciA[i]    = g_sys_context.g_motor_status[i].current_deciA;
        frame.comm_error[i]       = g_sys_context.g_motor_status[i].comm_error;
    }

    frame.tail[0] = 0x0D;
    frame.tail[1] = 0x0A;

    Debug_SendData((const uint8_t *)&frame, sizeof(frame));
}

// ========================== 三重系统安防保护检查 ==========================

/**
 * @brief  三重系统级安防防护检查（通信中断、同步差超限、过流堵转）
 * @return true: 触发故障急停; false: 系统安全
 */
static bool APP_Control_CheckSafety(void)
{
    uint8_t active_mask = App_Data_GetColumnMotorMask();

    // 1. 通信连续中断检查 (连续 10 帧/40ms 接收失败触发通信保护)
    for (int i = 0; i < 4; i++) {
        if (!(active_mask & (1 << i))) continue;
        if (g_sys_context.g_motor_status[i].comm_error >= SAFETY_COMM_ERR_MAX_CNT) {
            g_sys_context.system_fault_code = FAULT_CODE_COMM;
            Debug_Printf("[ERR] Safety Fault: Motor %d Comm Loss! (CommErr=%d)\r\n",
                         i + 1, g_sys_context.g_motor_status[i].comm_error);
            return true;
        }
    }

    // 2. 轴间真实绝对高度差超限检查 (常规长行程联动运行安防)
    if (g_sys_context.max_travel_diff > (float)g_sys_context.max_sync_diff_hall) {
        g_sys_context.system_fault_code = FAULT_CODE_SYNC;
        Debug_Printf("[ERR] Safety Fault: Sync Travel Diff Exceeded! (Diff=%.1f > Limit=%d)\r\n",
                     g_sys_context.max_travel_diff, g_sys_context.max_sync_diff_hall);
        Debug_Printf("[SYS] TravelRel: TR0=%.0f, TR1=%.0f, TR2=%.0f, TR3=%.0f | AbsHalls: H0=%d, H1=%d, H2=%d, H3=%d\r\n",
                     g_sys_context.travel_rel[0], g_sys_context.travel_rel[1], g_sys_context.travel_rel[2], g_sys_context.travel_rel[3],
                     g_sys_context.g_motor_status[0].current_abs_hall,
                     g_sys_context.g_motor_status[1].current_abs_hall,
                     g_sys_context.g_motor_status[2].current_abs_hall,
                     g_sys_context.g_motor_status[3].current_abs_hall);
        return true;
    }

    // 3. 单轴过流堵转检查
    for (int i = 0; i < 4; i++) {
        if (!(active_mask & (1 << i))) continue;
        if (g_sys_context.g_motor_status[i].current_deciA > app_data.stall_current_threshold) {
            g_sys_context.g_motor_status[i].stall_cnt++;
            if (g_sys_context.g_motor_status[i].stall_cnt >= SAFETY_STALL_MAX_CNT) {
                g_sys_context.system_fault_code = FAULT_CODE_STALL;
                Debug_Printf("[ERR] Safety Fault: Motor %d OverCurrent Stall! (Curr=%.2fA > Limit=%.2fA)\r\n",
                             i + 1, (float)g_sys_context.g_motor_status[i].current_deciA / 100.0f,
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
    uint8_t active_mask = App_Data_GetColumnMotorMask();

    // 1. 根据共享上下文中的 4 轴绝对高度最大偏差动态确定 PID 限幅 (Dynamic Output Limits)
    // 偏差 <= 50 counts (0.44mm): 100 RPM 低平稳限幅
    // 偏差 50~300 counts (0.44~2.66mm): 线性平滑放大至 100~600 RPM
    // 偏差 > 300 counts (> 2.66mm): 强力极速拉平模式 600 RPM
    float dynamic_out_max  = PID_OUT_MAX_LOW;
    float dynamic_iout_max = PID_IOUT_MAX_LOW;
    if (g_sys_context.max_travel_diff > PID_DIFF_HIGH_THRESHOLD) {
        dynamic_out_max  = PID_OUT_MAX_HIGH;
        dynamic_iout_max = PID_IOUT_MAX_HIGH;
    } else if (g_sys_context.max_travel_diff > PID_DIFF_LOW_THRESHOLD) {
        float ratio      = (g_sys_context.max_travel_diff - PID_DIFF_LOW_THRESHOLD) / (PID_DIFF_HIGH_THRESHOLD - PID_DIFF_LOW_THRESHOLD);
        dynamic_out_max  = PID_OUT_MAX_LOW + ratio * (PID_OUT_MAX_HIGH - PID_OUT_MAX_LOW);
        dynamic_iout_max = PID_IOUT_MAX_LOW + ratio * (PID_IOUT_MAX_HIGH - PID_IOUT_MAX_LOW);
    } else {
        dynamic_out_max  = PID_OUT_MAX_LOW;
        dynamic_iout_max = PID_IOUT_MAX_LOW;
    }

    for (int i = 0; i < 4; i++) {
        if (!(active_mask & (1 << i))) continue;
        motor_pids[i].Out_Max  = dynamic_out_max;
        motor_pids[i].Out_Min  = -dynamic_out_max;
        motor_pids[i].Iout_Max = dynamic_iout_max;
    }

    float calc_target_v[4];
    float max_v = -1e9f;

    // 2. 算出使能通道的理论 PID 调速结果 (直接引用全局上下文 g_sys_context 中无条件解算的绝对伸出高度)
    for (int i = 0; i < 4; i++) {
        if (!(active_mask & (1 << i))) {
            calc_target_v[i] = 0.0f;
            continue;
        }
        APP_PID_SetTarget(&motor_pids[i], g_sys_context.avg_travel);
        float delta_v = APP_PID_Calc(&motor_pids[i], g_sys_context.travel_rel[i]);

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

    // 3. 防饱和速度平移：若最高轴理论转速突破 MOTOR_MAX_RUN_RPM，全局向下平移溢出量，保护打满触顶
    float shift_offset = 0.0f;
    if (max_v > (float)MOTOR_MAX_RUN_RPM) {
        shift_offset = max_v - (float)MOTOR_MAX_RUN_RPM;
    }

    for (int i = 0; i < 4; i++) {
        if (!(active_mask & (1 << i))) {
            g_sys_context.g_motor_status[i].target_speed = 0;
            continue;
        }
        float target_v = calc_target_v[i] - shift_offset;

        // 限制在 MOTOR_MIN_RUN_RPM ~ MOTOR_MAX_RUN_RPM 之间
        if (target_v > (float)MOTOR_MAX_RUN_RPM) {
            target_v = (float)MOTOR_MAX_RUN_RPM;
        } else if (target_v < (float)MOTOR_MIN_RUN_RPM && g_sys_context.g_motor_status[i].target_cmd != CMD_STOP) {
            target_v = (float)MOTOR_MIN_RUN_RPM;
        }

        if (g_sys_context.g_motor_status[i].target_cmd == CMD_STOP) {
            g_sys_context.g_motor_status[i].target_speed = 0;
        } else {
            g_sys_context.g_motor_status[i].target_speed = (int16_t)target_v;
        }
    }
}

/**
 * @brief 根据 app_data 的最新物理配置重新计算系统控制参数
 */
void APP_Control_UpdateParamsFromAppData(void)
{
    uint16_t ratio = (app_data.reduction_ratio > 0) ? app_data.reduction_ratio : 30;
    uint16_t lead  = (app_data.lead_mm > 0) ? app_data.lead_mm : 8;
    uint16_t coef  = (app_data.hall_coef > 0) ? app_data.hall_coef : 30;

    g_sys_context.counts_per_mm      = (float)(ratio * coef) / (float)lead;                                      // count/mm
    g_sys_context.calc_base_rpm      = (int16_t)((app_data.target_speed_mm_min * ratio) / lead);                 // 对应基础转速
    g_sys_context.max_sync_diff_hall = (int32_t)roundf(app_data.max_sync_diff_mm * g_sys_context.counts_per_mm); // 最大同步差
    g_sys_context.max_travel_hall    = (int32_t)roundf(app_data.max_travel_range_mm * g_sys_context.counts_per_mm);

    // 同步刷新并清理未使能轴的历史状态，防止模式切换后未使能轴残留的错误计数触发待机报错
    uint8_t active_mask = App_Data_GetColumnMotorMask();
    for (int i = 0; i < 4; i++) {
        if (!(active_mask & (1 << i))) {
            g_sys_context.g_motor_status[i].comm_error         = 0;
            g_sys_context.g_motor_status[i].driver_status_word = 0;
            g_sys_context.g_motor_status[i].driver_fault_code  = 0;
            g_sys_context.g_motor_status[i].target_cmd         = CMD_STOP;
            g_sys_context.g_motor_status[i].target_speed       = 0;
            g_sys_context.g_motor_status[i].stall_cnt          = 0;
        }
    }
}

/**
 * @brief 检查所有 4 根立柱是否均低于或等于安装起点 (即 current_abs_hall <= min_mount_halls + 容差)
 * @return true: 全部立柱均处于底部起点; false: 至少有一根立柱高于起点
 */
bool APP_Control_IsAllColumnsAtBottom(void)
{
    // 允许 1.0mm 内的机械自锁齿隙与回弹微小容差 (保底 10 counts)
    int32_t tolerance_counts = (int32_t)roundf(1.0f * g_sys_context.counts_per_mm);
    if (tolerance_counts < 10) tolerance_counts = 10;

    for (int i = 0; i < 4; i++) {
        if (g_sys_context.g_motor_status[i].current_abs_hall > (app_data.min_mount_halls[i] + tolerance_counts)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 动作启动前强制锁死当前柱体模式 (动则强锁防呆门禁)
 */
void APP_Control_EnsureColumnModeLocked(void)
{
    if (app_data.column_mode_locked == 0) {
        app_data.column_mode_locked = 1;
        APP_Data_Storage();
        Debug_Printf("[SYS] Motion Triggered: Column Mode %d Force-Locked to Flash!\r\n", app_data.column_mode);
    }
}

// ===================================================================
// 逐轴独立停稳检测与停机管理状态
// ===================================================================
static uint8_t s_axis_stable_cnt[4]      = {0, 0, 0, 0};                                     // 轴停稳计数器
static bool s_axis_is_settled[4]         = {false, false, false, false};                     // 轴停稳标志
static uint32_t s_last_check_halls[4]    = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}; // 轴上次检测绝对高度
static bool s_total_tune_axis_done[4]    = {false, false, false, false};                     // 四柱微调逐轴步进完成标志
static uint16_t s_total_tune_timeout_cnt = 0;                                                // 四柱微调防卡死安全超时计数

/**
 * @brief 准备切入停机归档阶段，初始化各轴独立停稳跟踪状态
 * @param motor_mask 参与本次运动的电机掩码 (如单轴微调为 1<<m_idx, 四轴联动为 0x0F)
 * @param next_step  目标停机状态 (SYS_STEP_TOTAL_DONE 或 SYS_STEP_TUNE_DONE)
 */
static void APP_Control_PrepareStopSettling(uint8_t motor_mask, Motor_ctl_Step_t next_step)
{
    g_sys_context.active_motor_mask = motor_mask;
    g_sys_context.system_step       = next_step;

    // 453 抱闸控制：若是致命故障急停状态，立即锁定抱闸自锁
    if (next_step == SYS_STEP_FAULT_STOP) {
        MID_Brake_Lock();
    }

    for (int i = 0; i < 4; i++) {
        s_last_check_halls[i] = 0xFFFFFFFF;
        s_axis_stable_cnt[i]  = 0;
        if (motor_mask & (1 << i)) {
            s_axis_is_settled[i] = false; // 参与运动的轴需要检测停稳
        } else {
            s_axis_is_settled[i] = true; // 未参与运动的轴直接视为已就绪
        }
    }
}

/**
 * @brief 全面复位 Sys_Ctrl_Context_t 运行状态与电机绝对位置 (在恢复出厂设置或系统大重置时调用)
 */
void APP_Control_ResetSystemContext(void)
{
    // 1. 重新更新物理系数与目标 RPM
    APP_Control_UpdateParamsFromAppData();

    // 2. 刷新 4 轴运行内存中的绝对位置为 Flash 恢复后的初始安装位置
    for (int i = 0; i < 4; i++) {
        g_sys_context.g_motor_status[i].current_abs_hall = app_data.motor_abs_halls[i];
        g_sys_context.g_motor_status[i].stall_cnt        = 0;
        g_sys_context.g_motor_status[i].current_deciA    = 0;
        g_sys_context.g_motor_status[i].current_speed    = 0;
        g_sys_context.g_motor_status[i].target_speed     = 0;
    }

    // 3. 复位系统状态机、错误码与微调标志
    g_sys_context.system_fault_code = FAULT_CODE_NONE;
    g_sys_context.system_step       = SYS_STEP_READY;
    g_sys_context.active_motor_mask = App_Data_GetColumnMotorMask();
    g_sys_context.is_single_tuning  = false;
    g_sys_context.is_total_tuning   = false;
    g_sys_context.max_travel_diff   = 0;
    MID_Brake_Lock(); // 系统复位锁定抱闸自锁
}

/**
 * @brief 外部发起单轴微调
 */
void APP_Control_StartSingleTune(uint8_t m_idx)
{
    extern uint8_t dim1;
    if (g_sys_context.system_step != SYS_STEP_READY || m_idx >= 4) return;
    if (!Sys_Mode_CanRunMotion() || dim1 != 0) return; // 权威门禁：调参或锁定态下禁止微调
    uint8_t active_mask = App_Data_GetColumnMotorMask();
    if (!(active_mask & (1 << m_idx))) return; // 门禁：当前模式未启用的轴直接忽视

    // 动则强锁：发生任何位移控制动作前强制锁死当前柱体模式，防范未锁定态空中变动拓扑
    APP_Control_EnsureColumnModeLocked();

    for (int i = 0; i < 4; i++) {
        g_sys_context.g_motor_status[i].last_motion_cmd  = CMD_STOP;
        g_sys_context.g_motor_status[i].start_drive_hall = g_sys_context.g_motor_status[i].hall_value;
        g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
    }

    g_sys_context.active_motor_mask         = (uint8_t)(1 << m_idx);
    g_sys_context.is_single_tuning          = true; // 显式标记正在单轴微调
    g_sys_context.single_tune_motor_idx     = m_idx;
    g_sys_context.single_tune_start_hall    = g_sys_context.g_motor_status[m_idx].hall_value;
    g_sys_context.single_tune_orig_abs_hall = g_sys_context.g_motor_status[m_idx].current_abs_hall;
    g_sys_context.single_tune_target_counts = (uint32_t)roundf((float)app_data.single_tune_step_mm * g_sys_context.counts_per_mm);

    // 单轴微调速度也设定为正常速度的一半 (若过小则保底 100 RPM)
    uint16_t tune_rpm = g_sys_context.calc_base_rpm / 2;
    if (tune_rpm < 100) tune_rpm = 100;

    xQueueReset(g_motor_ctrl_queue);

    // 453 机械抱闸安全时序：先通电松开抱闸，延时 80ms 等待机械脱开
    MID_Brake_Release();
    vTaskDelay(pdMS_TO_TICKS(80));

    Motor_Ctrl_Msg_t speed_msg = {CMD_SET_SPEED, (uint8_t)(1 << m_idx), tune_rpm};
    Motor_Ctrl_Msg_t cmd_msg   = {(g_sys_context.single_tune_dir == 0) ? CMD_FORWARD : CMD_REVERSE, (uint8_t)(1 << m_idx), 0};

    g_sys_context.g_motor_status[m_idx].target_cmd   = cmd_msg.cmd_type;
    g_sys_context.g_motor_status[m_idx].target_speed = tune_rpm;
    g_sys_context.g_motor_status[m_idx].stall_cnt    = 0;

    xQueueSend(g_motor_ctrl_queue, &speed_msg, pdMS_TO_TICKS(10));
    xQueueSend(g_motor_ctrl_queue, &cmd_msg, pdMS_TO_TICKS(10));

    g_sys_context.system_step = SYS_STEP_SINGLE_TUNE;
    Sys_Mode_Set(SYS_MODE_MOTION); // 同步系统模式为运动态
    Debug_Printf("[SYS] Enter SINGLE_TUNE: Motor=%d, Dir=%s, Step=%dmm, TargetCounts=%d, Speed=%dRPM (Half Speed)\r\n",
                 m_idx + 1, (g_sys_context.single_tune_dir == 0) ? "UP" : "DOWN",
                 app_data.single_tune_step_mm, g_sys_context.single_tune_target_counts, tune_rpm);
}

/**
 * @brief 取消/提前结束单轴微调
 */
void APP_Control_CancelSingleTune(void)
{
    if (g_sys_context.system_step == SYS_STEP_SINGLE_TUNE) {
        uint8_t m_idx = g_sys_context.single_tune_motor_idx;
        xQueueReset(g_motor_ctrl_queue);
        Motor_Ctrl_Msg_t stop_msg                        = {CMD_STOP, (uint8_t)(1 << m_idx), 0};
        g_sys_context.g_motor_status[m_idx].target_cmd   = CMD_STOP;
        g_sys_context.g_motor_status[m_idx].target_speed = 0;

        APP_Control_PrepareStopSettling((uint8_t)(1 << m_idx), SYS_STEP_TUNE_DONE);
        xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 外部发起四柱一键同步微调 (待机态长按 K5 触发)
 */
void APP_Control_StartTotalTune(void)
{
    if (g_sys_context.system_step != SYS_STEP_READY) return;
    if (!Sys_Mode_CanRunMotion()) return; // 权威门禁：调参或锁定态下禁止微调

    uint8_t tune_dir    = Sys_View_GetTuneDir(); // 0: 正转/上升, 1: 反转/下降
    uint8_t active_mask = App_Data_GetColumnMotorMask();

    // 启动前行程边界软限位安全检查：
    // 仅当向上微调时检查行程上限；向下微调放行（微调本身用于零点校准与调平，允许在零点向下微调）
    if (tune_dir == 0) {
        for (int i = 0; i < 4; i++) {
            if (!(active_mask & (1 << i))) continue;
            float tr = (float)(g_sys_context.g_motor_status[i].current_abs_hall - app_data.min_mount_halls[i]);
            if (tr >= (float)g_sys_context.max_travel_hall) {
                APP_Menu_SetPrompt("-Hi-", 20);
                MID_Buzzer_TriggerBeep(100);
                Debug_Printf("[SYS] Total Tune UP Blocked! Motor %d reached max travel (%.1f >= %d)\r\n",
                             i + 1, tr, g_sys_context.max_travel_hall);
                return;
            }
        }
    }

    // 动则强锁：发生任何位移控制动作前强制锁死当前柱体模式，防范未锁定态空中变动拓扑
    APP_Control_EnsureColumnModeLocked();

    uint32_t step_mm       = (app_data.single_tune_step_mm > 0) ? app_data.single_tune_step_mm : 1;
    uint32_t target_counts = (uint32_t)roundf((float)step_mm * g_sys_context.counts_per_mm);
    if (target_counts == 0) target_counts = 1;

    // 微调设定为正常速度的一半 (与单轴微调严格对齐，保底 100 RPM)
    uint16_t tune_rpm = g_sys_context.calc_base_rpm / 2;
    if (tune_rpm < 100) tune_rpm = 100;

    uint8_t motion_cmd = (tune_dir == 0) ? CMD_FORWARD : CMD_REVERSE;

    for (int i = 0; i < 4; i++) {
        s_total_tune_axis_done[i]                        = !(active_mask & (1 << i));
        g_sys_context.g_motor_status[i].last_motion_cmd  = motion_cmd;
        g_sys_context.g_motor_status[i].start_drive_hall = g_sys_context.g_motor_status[i].hall_value;
        g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
        g_sys_context.g_motor_status[i].target_cmd       = (active_mask & (1 << i)) ? motion_cmd : CMD_STOP;
        g_sys_context.g_motor_status[i].target_speed     = (active_mask & (1 << i)) ? tune_rpm : 0;
        g_sys_context.g_motor_status[i].stall_cnt        = 0;
        g_sys_context.delta_h[i]                         = 0.0f;
    }
    s_total_tune_timeout_cnt = 0;

    g_sys_context.avg_delta_h              = 0.0f;
    g_sys_context.max_dh_diff              = 0.0f;
    g_sys_context.active_motor_mask        = active_mask;
    g_sys_context.is_single_tuning         = false;
    g_sys_context.is_total_tuning          = true;
    g_sys_context.total_tune_target_counts = target_counts;
    g_sys_context.base_speed               = tune_rpm;
    g_sys_context.ramp_cnt                 = RAMP_UP_TOTAL_STEPS;
    g_sys_context.system_step              = SYS_STEP_TOTAL_TUNE; // 独立状态：切入同步微调专用状态
    Sys_Mode_Set(SYS_MODE_MOTION);                                // 在抱闸脱开延时前先切入运动态，使路由层在延时期间也能准确识别运动态并过滤按键

    // 453 机械抱闸安全时序：启动前先通电松开抱闸，延时 80ms 等待机械脱开
    MID_Brake_Release();
    vTaskDelay(pdMS_TO_TICKS(80));

    xQueueReset(g_motor_ctrl_queue);

    Motor_Ctrl_Msg_t speed_msg = {CMD_SET_SPEED, active_mask, tune_rpm};
    Motor_Ctrl_Msg_t cmd_msg   = {(Motor_Cmd_Type_t)motion_cmd, active_mask, 0};

    xQueueSend(g_motor_ctrl_queue, &speed_msg, pdMS_TO_TICKS(10));
    xQueueSend(g_motor_ctrl_queue, &cmd_msg, pdMS_TO_TICKS(10));

    if (tune_dir == 0) {
        APP_Menu_SetPrompt("-AU-", 25);
    } else {
        APP_Menu_SetPrompt("-Ad-", 25);
    }
    MID_Buzzer_TriggerBeep(40);

    Debug_Printf("[SYS] Enter TOTAL_TUNE: Mask=0x%02X, Dir=%s, Step=%dmm, TargetCounts=%d, Speed=%dRPM (Half Speed)\r\n",
                 active_mask, (tune_dir == 0) ? "UP" : "DOWN", step_mm, target_counts, tune_rpm);
}

/**
 * @brief 系统紧急停止 (Direct Action，直接动作切断输出并停机)
 */
void APP_Control_EmergencyStop(void)
{
    uint8_t active_mask = App_Data_GetColumnMotorMask();

    // 1. 清空命令队列，防止排队旧指令发出
    xQueueReset(g_motor_ctrl_queue);

    // 2. 立即下发使能轴停机指令
    Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, active_mask, 0};
    xQueueSend(g_motor_ctrl_queue, &stop_msg, 0);

    for (int i = 0; i < 4; i++) {
        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
        g_sys_context.g_motor_status[i].target_speed = 0;
    }

    g_sys_context.is_single_tuning = false;
    g_sys_context.is_total_tuning  = false;

    // 3. 切入停机归档
    if (g_sys_context.system_step == SYS_STEP_TOTAL_RUNNING ||
        g_sys_context.system_step == SYS_STEP_AUTO_ALIGN ||
        g_sys_context.system_step == SYS_STEP_SINGLE_TUNE ||
        g_sys_context.system_step == SYS_STEP_TOTAL_REBOUND) {
        APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
    }
    Debug_Printf("[SYS] Emergency Stop Executed! All Active Motors Stopped.\r\n");
}

/**
 * @brief 用户按键确认清除故障急停状态并下发自由停机归档
 * @note 仅在通信正常且驱动器故障已恢复应答后开放调用
 * @return true: 成功清除并下发自由停机归档; false: 硬件通信或故障仍存在，忽略清除
 */
bool APP_Control_ClearFault(void)
{
    if (g_sys_context.system_step != SYS_STEP_FAULT_STOP) {
        return false;
    }

    uint8_t active_mask = App_Data_GetColumnMotorMask();

    // 1. 严格核验使能轴 485 通信状态是否完全恢复正常
    for (int i = 0; i < 4; i++) {
        if (!(active_mask & (1 << i))) continue;
        if (g_sys_context.g_motor_status[i].comm_error > 0) {
            Debug_Printf("[SYS] ClearFault blocked: Motor %d comm not ready!\r\n", i + 1);
            return false;
        }
    }

    // 2. 严格核验使能轴驱动器本体是否已确认解除故障红灯
    for (int i = 0; i < 4; i++) {
        if (!(active_mask & (1 << i))) continue;
        if (g_sys_context.g_motor_status[i].driver_status_word == 0x0004) {
            Debug_Printf("[SYS] ClearFault blocked: Motor %d still in Driver Fault (Status=0x%04X, Code=%d)\r\n",
                         i + 1, g_sys_context.g_motor_status[i].driver_status_word,
                         g_sys_context.g_motor_status[i].driver_fault_code);
            return false;
        }
    }

    xQueueReset(g_motor_ctrl_queue);

    for (int i = 0; i < 4; i++) {
        g_sys_context.g_motor_status[i].target_cmd   = CMD_IDLE_STOP;
        g_sys_context.g_motor_status[i].target_speed = 0;
    }

    // 收到应答后开放按钮：下发自由停机 (CMD_IDLE_STOP: 写 0x2000 = 0x0005)
    Motor_Ctrl_Msg_t idle_stop_msg = {CMD_IDLE_STOP, active_mask, 0};
    xQueueSend(g_motor_ctrl_queue, &idle_stop_msg, pdMS_TO_TICKS(10));

    // 清除主控故障代码
    g_sys_context.system_fault_code = FAULT_CODE_NONE;

    // 进入停稳收尾与位置固化阶段 (Flash 归档绝对位置，抱闸抱紧，回到 READY)
    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
    Debug_Printf("[SYS] Fault Lockout Cleared by User Key! Sent CMD_IDLE_STOP (0x0005) Free Stop...\r\n");
    return true;
}

/**
 * @brief 核心控制解算：统一解算 4 轴绝对高度、位移增量 ΔH、绝对伸出行程 travel_rel 及极差统计量
 * @note 严格采用物理脉冲绝对值结合电机运动方向符号，彻底解决倒转/反弹时的负数突变与极差暴涨问题
 */
void APP_Control_UpdateStateAndStatistics(void)
{
    float min_dh = 1e9f, max_dh = -1e9f;
    float min_tr = 1e9f, max_tr = -1e9f;

    g_sys_context.avg_delta_h = 0.0f;
    g_sys_context.avg_travel  = 0.0f;
    uint8_t active_mask       = App_Data_GetColumnMotorMask();
    uint8_t active_count      = 0;

    for (int i = 0; i < 4; i++) {
        uint32_t now_hall   = g_sys_context.g_motor_status[i].hall_value;
        uint32_t start_hall = g_sys_context.g_motor_status[i].start_drive_hall;

        // 1. 32 位无符号补码减法 + 绝对值：求解本阶段运动的物理霍尔脉冲数 (免疫 0/0xFFFFFFFF 绕回)
        int32_t raw_diff   = (int32_t)(now_hall - start_hall);
        uint32_t abs_pulse = (raw_diff >= 0) ? (uint32_t)raw_diff : (uint32_t)(-raw_diff);

        // 2. 根据该轴有效运动方向，求解带有物理高度方向的 signed_delta
        uint8_t cmd          = g_sys_context.g_motor_status[i].target_cmd;
        int32_t signed_delta = 0;

        if (cmd == CMD_FORWARD) {
            g_sys_context.g_motor_status[i].last_motion_cmd = CMD_FORWARD;
            signed_delta                                    = (int32_t)abs_pulse; // 上升：绝对高度增加
        } else if (cmd == CMD_REVERSE) {
            g_sys_context.g_motor_status[i].last_motion_cmd = CMD_REVERSE;
            signed_delta                                    = -(int32_t)abs_pulse; // 下降：绝对高度减少
        } else {
            // CMD_STOP 停机/静止阶段：使用带符号的物理脉冲差 (raw_diff) 并结合丝杆极性映射为高度方向
            signed_delta = (app_data.motor_dir_invert == 0) ? raw_diff : -raw_diff;
        }

        // 3. 求解当前绝对高度 (起点高度 + 方向增量)
        int32_t calc_abs_hall                            = g_sys_context.g_motor_status[i].base_abs_hall + signed_delta;
        g_sys_context.g_motor_status[i].current_abs_hall = calc_abs_hall;

        // A. 本次运动过程中的位移增量 ΔH_i
        g_sys_context.delta_h[i] = (float)signed_delta;

        // B. 基于调平零点 (min_mount_halls) 的真正物理伸出行程 travel_rel[i] (用于 PID 闭环纠偏)
        g_sys_context.travel_rel[i] = (float)(calc_abs_hall - app_data.min_mount_halls[i]);

        if (active_mask & (1 << i)) {
            active_count++;
            g_sys_context.avg_delta_h += g_sys_context.delta_h[i];
            if (g_sys_context.delta_h[i] < min_dh) min_dh = g_sys_context.delta_h[i];
            if (g_sys_context.delta_h[i] > max_dh) max_dh = g_sys_context.delta_h[i];

            g_sys_context.avg_travel += g_sys_context.travel_rel[i];
            if (g_sys_context.travel_rel[i] < min_tr) min_tr = g_sys_context.travel_rel[i];
            if (g_sys_context.travel_rel[i] > max_tr) max_tr = g_sys_context.travel_rel[i];
        }
    }

    if (active_count > 0) {
        g_sys_context.avg_delta_h /= (float)active_count;
        g_sys_context.max_dh_diff = max_dh - min_dh; // 本次单次运动位移增量极差

        g_sys_context.avg_travel /= (float)active_count;
        g_sys_context.max_travel_diff = max_tr - min_tr; // 调平伸出行程极差 (伸出差)
    } else {
        g_sys_context.max_dh_diff     = 0.0f;
        g_sys_context.max_travel_diff = 0.0f;
    }
}

// ========================== 核心业务控制任务 ==========================

/**
 * @brief 核心业务控制任务（解耦数据驱动架构）
 */
void APP_ControlTask(void *pvParameters)
{
    MID_SIGNAL_Msg sig_msg;
    static int16_t last_sent_speed[4] = {-1, -1, -1, -1};

    // 1. 初始化系统上下文与 PID (匹配 5ms/200Hz 极速控制)
    g_sys_context.system_step       = SYS_STEP_Boot;
    g_sys_context.is_hardware_ready = false;
    g_sys_context.active_motor_mask = App_Data_GetColumnMotorMask();
    g_sys_context.is_single_tuning  = false;
    g_sys_context.is_total_tuning   = false;
    g_sys_context.base_speed        = 0;
    g_sys_context.system_fault_code = 0;

    for (int i = 0; i < 4; i++) {
        g_sys_context.g_motor_status[i].driver_status_word = 0;
        g_sys_context.g_motor_status[i].driver_fault_code  = 0;
        APP_PID_Init(&motor_pids[i], PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD,
                     PID_DEFAULT_DEADZONE, PID_DEFAULT_OUT_MAX, PID_DEFAULT_OUT_MIN, PID_DEFAULT_IOUT_MAX);
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        uint8_t active_mask = App_Data_GetColumnMotorMask();

        // 非阻塞检查信号/事件 (优先从 LEPA 单值覆盖邮箱获取，防积压；向下兼容旧队列)
        bool has_event = false;
        uint8_t mb_src = 0, mb_id = 0, mb_evt = 0;
        if (Sys_Mailbox_GetMotionCmd(&mb_src, &mb_id, &mb_evt)) {
            sig_msg.signal_id = (MID_Signal_ID)mb_id;
            sig_msg.event     = (MID_Signal_EventType)mb_evt;
            has_event         = true;
        } else if (MID_Signal_GetEvent(&sig_msg, 0) == pdTRUE) {
            mb_src    = SYS_MOTION_SRC_SIGNAL;
            mb_id     = (uint8_t)sig_msg.signal_id;
            mb_evt    = (uint8_t)sig_msg.event;
            has_event = true;
        }

        switch (g_sys_context.system_step) {
            // === Boot 状态：等待硬件初始化完成并执行物理参数换算 ===
            case SYS_STEP_Boot: {
                if (g_sys_context.is_hardware_ready) {
                    // 同步 Flash 保存的高度绝对起点至监控内存，初始化起点与行程
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].current_abs_hall = app_data.motor_abs_halls[i];
                        g_sys_context.g_motor_status[i].base_abs_hall    = app_data.motor_abs_halls[i];
                        g_sys_context.g_motor_status[i].start_drive_hall = g_sys_context.g_motor_status[i].hall_value;
                        g_sys_context.travel_rel[i]                      = (float)(app_data.motor_abs_halls[i] - app_data.min_mount_halls[i]);
                    }

                    // 物理参数单位自动换算
                    APP_Control_UpdateParamsFromAppData();

                    g_sys_context.system_step = SYS_STEP_READY;
                    Debug_Printf("[SYS] Counts/mm=%.1f, CalcRPM=%d, MaxSyncDiffHall=%d, MaxTravelHall=%d\r\n",
                                 g_sys_context.counts_per_mm, g_sys_context.calc_base_rpm,
                                 g_sys_context.max_sync_diff_hall, g_sys_context.max_travel_hall);
                }
                break;
            }

            // === READY 状态：就绪待命 ===
            case SYS_STEP_READY: {
                // 门禁检查：若系统处于菜单调参或深度调试层中 (dim1 != 0 或 SYS_MODE_MENU_CONFIG)，绝不执行待机通信中断报错，且安全清空微调按键
                extern uint8_t dim1;
                if (Sys_Mode_IsMenuActive() || dim1 != 0) {
                    Sys_Mailbox_ClearMotionCmd();
                    break;
                }

                // 0. 待机状态实时 485 通信健康度巡检 (仅针对使能轴)
                bool has_comm_loss = false;
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    if (g_sys_context.g_motor_status[i].comm_error >= SAFETY_COMM_ERR_MAX_CNT) {
                        has_comm_loss = true;
                        Debug_Printf("[ERR] Standby Comm Loss! Motor %d (CommErr=%d >= Limit=%d)\r\n",
                                     i + 1, g_sys_context.g_motor_status[i].comm_error, SAFETY_COMM_ERR_MAX_CNT);
                        break;
                    }
                }
                if (has_comm_loss) {
                    g_sys_context.system_fault_code = FAULT_CODE_COMM;
                    g_sys_context.system_step       = SYS_STEP_FAULT_STOP;
                    break;
                }

                // 1. 待机状态驱动器本体报警巡检 (仅针对使能轴，0x2100 状态字 == 0x0004 驱动器故障中)
                bool has_driver_alarm = false;
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    if (g_sys_context.g_motor_status[i].driver_status_word == 0x0004) {
                        has_driver_alarm = true;
                        Debug_Printf("[ERR] Standby Driver Alarm! Motor %d: Status=0x%04X, FaultCode=%d\r\n",
                                     i + 1, g_sys_context.g_motor_status[i].driver_status_word,
                                     g_sys_context.g_motor_status[i].driver_fault_code);
                        break;
                    }
                }
                if (has_driver_alarm) {
                    g_sys_context.system_fault_code = FAULT_CODE_DRIVER_ALARM; // 置 Err6 驱动器本体报警
                    g_sys_context.system_step       = SYS_STEP_FAULT_STOP;
                    break;
                }

                // 2. 检查面板按键直发微调 (K1 ~ K4 单轴微调，K5 长按四柱/双柱一键同步微调)
                if (has_event && mb_src == SYS_MOTION_SRC_KEY) {
                    if (mb_id <= MID_KEY_ID_K4 && mb_evt == MID_KEY_EVT_LEASS) {
                        uint8_t m_idx = (uint8_t)(mb_id - MID_KEY_ID_K1);
                        if (!(active_mask & (1 << m_idx))) {
                            // 未使能轴直接忽视，不用报警
                            break;
                        }
                        g_sys_context.single_tune_dir = Sys_View_GetTuneDir(); // 同步设定的微调方向
                        APP_Control_StartSingleTune(m_idx);
                        Debug_Printf("[SYS] Autonomously Starting Single Tune on Motor %d via K%d!\r\n", m_idx + 1, m_idx + 1);
                        break;
                    } else if (mb_id == MID_KEY_ID_K5 && mb_evt == MID_KEY_EVT_LONG) {
                        APP_Control_StartTotalTune();
                        for (int i = 0; i < 4; i++) {
                            last_sent_speed[i] = g_sys_context.base_speed;
                        }
                        Sys_Mailbox_ClearMotionCmd(); // 启动后清空邮箱，防止长按残余事件进入运行态干扰
                        Debug_Printf("[SYS] Autonomously Starting Total Tune via K5 Long Press!\r\n");
                        break;
                    }
                }

                // 3. 检查遥控信号触发下行 / 上行
                if (has_event && mb_src == SYS_MOTION_SRC_SIGNAL && sig_msg.event == MID_SIGNAL_EVT_TRIGGER) {
                    Motor_Ctrl_Msg_t speed_msg;
                    Motor_Ctrl_Msg_t cmd_msg;
                    bool action_valid = false;

                    int16_t run_rpm = (g_sys_context.calc_base_rpm > 0) ? g_sys_context.calc_base_rpm : 2400;

                    switch (sig_msg.signal_id) {
                        case MID_SIGNAL_REMOT_3:
                        case MID_SIGNAL_BUTON_DW: { // B 键 (REMOT_3) & 外接信号下行 (PC7)
                            // 权威门禁检查：调参中或故障锁定下严禁启动运动
                            if (!Sys_Mode_CanRunMotion()) {
                                Debug_Printf("[SYS] Down Start Blocked by Supervisor Gate!\r\n");
                                break;
                            }

                            // 启动前安全检查：如果有使能轴已到达或低于底部零点 (travel_rel <= 0)，禁止启动下行！
                            bool limit_blocked = false;
                            for (int i = 0; i < 4; i++) {
                                if (!(active_mask & (1 << i))) continue;
                                float tr = (float)(g_sys_context.g_motor_status[i].current_abs_hall - app_data.min_mount_halls[i]);
                                if (tr <= 0.0f) {
                                    limit_blocked = true;
                                    Debug_Printf("[SYS] Down Start Blocked! Motor %d reached bottom limit (Travel=%.1f <= 0)\r\n", i + 1, tr);
                                    break;
                                }
                            }
                            if (limit_blocked) break;

                            // 动则强锁：发生任何位移控制动作前强制锁死当前柱体模式，防范未锁定态空中变动拓扑
                            APP_Control_EnsureColumnModeLocked();

                            g_sys_context.is_single_tuning  = false;
                            g_sys_context.is_total_tuning   = false;
                            g_sys_context.active_motor_mask = active_mask;
                            g_sys_context.base_speed        = run_rpm;

                            speed_msg.cmd_type   = CMD_SET_SPEED;
                            speed_msg.motor_mask = active_mask;
                            speed_msg.speed_rpm  = 300; // 起点转速 300 RPM

                            cmd_msg.cmd_type   = CMD_REVERSE;
                            cmd_msg.motor_mask = active_mask;
                            cmd_msg.speed_rpm  = 0;

                            for (int i = 0; i < 4; i++) {
                                g_sys_context.g_motor_status[i].start_drive_hall = g_sys_context.g_motor_status[i].hall_value;
                                g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                                g_sys_context.g_motor_status[i].target_cmd       = CMD_REVERSE;
                                g_sys_context.g_motor_status[i].target_speed     = 300;
                                g_sys_context.g_motor_status[i].stall_cnt        = 0;
                                g_sys_context.delta_h[i]                         = 0.0f;
                                last_sent_speed[i]                               = 300;
                            }
                            g_sys_context.avg_delta_h = 0.0f;
                            g_sys_context.max_dh_diff = 0.0f;

                            // 453 机械抱闸安全时序：启动前先通电松开抱闸，延时 80ms 等待机械脱开
                            MID_Brake_Release();
                            vTaskDelay(pdMS_TO_TICKS(80));

                            xQueueReset(g_motor_ctrl_queue);
                            xQueueSend(g_motor_ctrl_queue, &speed_msg, pdMS_TO_TICKS(10));
                            xQueueSend(g_motor_ctrl_queue, &cmd_msg, pdMS_TO_TICKS(10));

                            action_valid              = true;
                            g_sys_context.ramp_cnt    = 0; // 重置 1000ms 缓启动计数
                            g_sys_context.system_step = SYS_STEP_TOTAL_RUNNING;
                            Sys_Mode_Set(SYS_MODE_MOTION); // 同步系统模式为运动态
                            break;
                        }

                        case MID_SIGNAL_REMOT_4:
                        case MID_SIGNAL_BUTON_UP: { // 遥控上行 & 外接信号上行 (PC8)
                            // 权威门禁检查：调参中或故障锁定下严禁启动运动
                            if (!Sys_Mode_CanRunMotion()) {
                                Debug_Printf("[SYS] Up Start Blocked by Supervisor Gate!\r\n");
                                break;
                            }

                            // 启动前安全检查：如果有使能轴已到达或超过最大行程上限 (travel_rel >= max_travel_hall)，禁止启动上行！
                            bool limit_blocked = false;
                            for (int i = 0; i < 4; i++) {
                                if (!(active_mask & (1 << i))) continue;
                                float tr = (float)(g_sys_context.g_motor_status[i].current_abs_hall - app_data.min_mount_halls[i]);
                                if (tr >= (float)g_sys_context.max_travel_hall) {
                                    limit_blocked = true;
                                    Debug_Printf("[SYS] Up Start Blocked! Motor %d reached max travel limit (Travel=%.1f >= Limit=%d)\r\n",
                                                 i + 1, tr, g_sys_context.max_travel_hall);
                                    break;
                                }
                            }
                            if (limit_blocked) break;

                            // 动则强锁：发生任何位移控制动作前强制锁死当前柱体模式，防范未锁定态空中变动拓扑
                            APP_Control_EnsureColumnModeLocked();

                            g_sys_context.is_single_tuning  = false;
                            g_sys_context.is_total_tuning   = false;
                            g_sys_context.active_motor_mask = active_mask;
                            g_sys_context.base_speed        = run_rpm;

                            speed_msg.cmd_type   = CMD_SET_SPEED;
                            speed_msg.motor_mask = active_mask;
                            speed_msg.speed_rpm  = 300; // 起点转速 300 RPM

                            cmd_msg.cmd_type   = CMD_FORWARD;
                            cmd_msg.motor_mask = active_mask;
                            cmd_msg.speed_rpm  = 0;

                            for (int i = 0; i < 4; i++) {
                                g_sys_context.g_motor_status[i].start_drive_hall = g_sys_context.g_motor_status[i].hall_value;
                                g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                                g_sys_context.g_motor_status[i].target_cmd       = CMD_FORWARD;
                                g_sys_context.g_motor_status[i].target_speed     = 300;
                                g_sys_context.g_motor_status[i].stall_cnt        = 0;
                                g_sys_context.delta_h[i]                         = 0.0f;
                                last_sent_speed[i]                               = 300;
                            }
                            g_sys_context.avg_delta_h = 0.0f;
                            g_sys_context.max_dh_diff = 0.0f;

                            // 453 机械抱闸安全时序：启动前先通电松开抱闸，延时 80ms 等待机械脱开
                            MID_Brake_Release();
                            vTaskDelay(pdMS_TO_TICKS(80));

                            xQueueReset(g_motor_ctrl_queue);
                            xQueueSend(g_motor_ctrl_queue, &speed_msg, pdMS_TO_TICKS(10));
                            xQueueSend(g_motor_ctrl_queue, &cmd_msg, pdMS_TO_TICKS(10));

                            action_valid              = true;
                            g_sys_context.ramp_cnt    = 0; // 重置 1000ms 缓启动计数
                            g_sys_context.system_step = SYS_STEP_TOTAL_RUNNING;
                            Sys_Mode_Set(SYS_MODE_MOTION); // 同步系统模式为运动态
                            break;
                        }

                        default:
                            break;
                    }

                    if (action_valid) {
                        Sys_Mailbox_ClearMotionCmd();
                        MID_SIGNAL_Msg dummy_msg;
                        while (MID_Signal_GetEvent(&dummy_msg, 0) == pdTRUE);
                    }
                }
                break;
            }

            // === SYS_STEP_SINGLE_TUNE 阶段：单轴微调运行中 ===
            case SYS_STEP_SINGLE_TUNE: {
                uint8_t m_idx       = g_sys_context.single_tune_motor_idx;
                uint32_t now_hall   = g_sys_context.g_motor_status[m_idx].hall_value;
                uint32_t start_hall = g_sys_context.single_tune_start_hall;

                // 0. 检查用户输入打断 (物理面板按键或遥控急停 A 键刹停取消微调)
                if (has_event) {
                    if (mb_src == SYS_MOTION_SRC_KEY || (mb_src == SYS_MOTION_SRC_SIGNAL && sig_msg.signal_id == MID_SIGNAL_REMOT_2)) {
                        APP_Control_CancelSingleTune();
                        Debug_Printf("[SYS] Single Tune Interrupted Autonomously by Input Event!\r\n");
                        break;
                    }
                }

                // 1. 安防防线一：通信中断检查 (若任一轴掉线，立即紧急停机)
                bool has_comm_loss = false;
                for (int i = 0; i < 4; i++) {
                    if (g_sys_context.g_motor_status[i].comm_error >= SAFETY_COMM_ERR_MAX_CNT) {
                        has_comm_loss = true;
                        Debug_Printf("[ERR] Single Tune Comm Loss! Motor %d (CommErr=%d >= Limit=%d)\r\n",
                                     i + 1, g_sys_context.g_motor_status[i].comm_error, SAFETY_COMM_ERR_MAX_CNT);
                        break;
                    }
                }
                if (has_comm_loss) {
                    xQueueReset(g_motor_ctrl_queue);
                    Motor_Ctrl_Msg_t stop_msg                        = {CMD_STOP, (uint8_t)(1 << m_idx), 0};
                    g_sys_context.g_motor_status[m_idx].target_cmd   = CMD_STOP;
                    g_sys_context.g_motor_status[m_idx].target_speed = 0;
                    g_sys_context.system_fault_code                  = FAULT_CODE_COMM;

                    APP_Control_PrepareStopSettling((uint8_t)(1 << m_idx), SYS_STEP_TUNE_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    break;
                }

                // 2. 安防防线二：微调目标轴过流堵转检查
                if (g_sys_context.g_motor_status[m_idx].current_deciA > app_data.stall_current_threshold) {
                    g_sys_context.g_motor_status[m_idx].stall_cnt++;
                    if (g_sys_context.g_motor_status[m_idx].stall_cnt >= SAFETY_STALL_MAX_CNT) {
                        xQueueReset(g_motor_ctrl_queue);
                        Motor_Ctrl_Msg_t stop_msg                        = {CMD_STOP, (uint8_t)(1 << m_idx), 0};
                        g_sys_context.g_motor_status[m_idx].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[m_idx].target_speed = 0;
                        g_sys_context.system_fault_code                  = FAULT_CODE_STALL;

                        APP_Control_PrepareStopSettling((uint8_t)(1 << m_idx), SYS_STEP_TUNE_DONE);
                        xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                        Debug_Printf("[ERR] Single Tune OverCurrent Stall! Motor %d (Curr=%.2fA > Limit=%.2fA). Stopping & Archiving...\r\n",
                                     m_idx + 1, (float)g_sys_context.g_motor_status[m_idx].current_deciA / 100.0f,
                                     (float)app_data.stall_current_threshold / 100.0f);
                        break;
                    }
                } else {
                    if (g_sys_context.g_motor_status[m_idx].stall_cnt > 0) {
                        g_sys_context.g_motor_status[m_idx].stall_cnt--;
                    }
                }

                // 若驱动器状态命令尚未 ACK 匹配成功，在 20ms 周期内持续补发，强保驱动器启动
                if (g_sys_context.g_motor_status[m_idx].current_cmd != g_sys_context.g_motor_status[m_idx].target_cmd ||
                    g_sys_context.g_motor_status[m_idx].current_speed != g_sys_context.g_motor_status[m_idx].target_speed) {
                    Motor_Ctrl_Msg_t speed_msg = {CMD_SET_SPEED, (uint8_t)(1 << m_idx), g_sys_context.g_motor_status[m_idx].target_speed};
                    Motor_Ctrl_Msg_t cmd_msg   = {(Motor_Cmd_Type_t)g_sys_context.g_motor_status[m_idx].target_cmd, (uint8_t)(1 << m_idx), 0};
                    xQueueSend(g_motor_ctrl_queue, &speed_msg, 0);
                    xQueueSend(g_motor_ctrl_queue, &cmd_msg, 0);
                }

                // 32 位无符号补码减法计算微调产生的绝对霍尔变动脉冲
                int32_t raw_delta  = (int32_t)(now_hall - start_hall);
                uint32_t abs_delta = (raw_delta >= 0) ? (uint32_t)raw_delta : (uint32_t)(-raw_delta);

                // 根据微调方向 (0: 正转/上升, 1: 反转/下降) 精确求解物理高度符号增量
                int32_t signed_delta = (g_sys_context.single_tune_dir == 0) ? (int32_t)abs_delta : -(int32_t)abs_delta;

                // 解算微调轴最新精确绝对高度与行程
                g_sys_context.g_motor_status[m_idx].current_abs_hall = g_sys_context.single_tune_orig_abs_hall + signed_delta;
                g_sys_context.travel_rel[m_idx]                      = (float)(g_sys_context.g_motor_status[m_idx].current_abs_hall - app_data.min_mount_halls[m_idx]);

                // 若达到目标步计数，停止该轴运动并切入 SYS_STEP_TUNE_DONE 归档
                if (abs_delta >= g_sys_context.single_tune_target_counts) {
                    xQueueReset(g_motor_ctrl_queue);

                    Motor_Ctrl_Msg_t stop_msg                        = {CMD_STOP, (uint8_t)(1 << m_idx), 0};
                    g_sys_context.g_motor_status[m_idx].target_cmd   = CMD_STOP;
                    g_sys_context.g_motor_status[m_idx].target_speed = 0;

                    APP_Control_PrepareStopSettling((uint8_t)(1 << m_idx), SYS_STEP_TUNE_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[SYS] Single Tune Reached Target Counts (%d >= %d), Stopping Motor %d...\r\n",
                                 abs_delta, g_sys_context.single_tune_target_counts, m_idx + 1);
                }

                APP_Control_DebugPrint();
                break;
            }

            // === SYS_STEP_TOTAL_TUNE 阶段：四柱一键同步微调运行中 (独立专用状态) ===
            case SYS_STEP_TOTAL_TUNE: {
                // 0. 更新 4 轴状态与解算 (刷新霍尔与位移)
                APP_Control_UpdateStateAndStatistics();

                // 1. 用户按键打断检查：外部面板按键短按或遥控 C 键停止，立即就地刹停取消微调！
                // (注意：K5 连发与释放已在路由层隔离，微调中若收到非 K5 按键或遥控急停直接制动)
                if (has_event) {
                    if ((mb_src == SYS_MOTION_SRC_SIGNAL && sig_msg.signal_id == MID_SIGNAL_REMOT_2) ||
                        (mb_src == SYS_MOTION_SRC_KEY && mb_id != MID_KEY_ID_K5)) {
                        xQueueReset(g_motor_ctrl_queue);
                        for (int i = 0; i < 4; i++) {
                            g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                            g_sys_context.g_motor_status[i].target_speed = 0;
                            last_sent_speed[i]                           = 0;
                        }
                        Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, active_mask, 0};
                        APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                        xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                        Debug_Printf("[SYS] Total Tune Interrupted Autonomously by User Event! Stopping...\r\n");
                        break;
                    }
                }

                // 2. 微调专属安防检查（直接急停锁死，绝不反弹）
                // 防线一：485 通信中断检查
                bool has_comm_loss = false;
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    if (g_sys_context.g_motor_status[i].comm_error >= SAFETY_COMM_ERR_MAX_CNT) {
                        has_comm_loss = true;
                        Debug_Printf("[ERR] Total Tune Comm Loss! Motor %d (CommErr=%d >= Limit=%d)\r\n",
                                     i + 1, g_sys_context.g_motor_status[i].comm_error, SAFETY_COMM_ERR_MAX_CNT);
                        break;
                    }
                }
                if (has_comm_loss) {
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg       = {CMD_STOP, active_mask, 0};
                    g_sys_context.system_fault_code = FAULT_CODE_COMM; // 置 Err2 通信故障
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    break;
                }

                // 防线二：单轴过流堵转检查 (直接急停锁定抱闸自锁，绝不反弹)
                bool has_stall = false;
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    if (g_sys_context.g_motor_status[i].current_deciA > app_data.stall_current_threshold) {
                        g_sys_context.g_motor_status[i].stall_cnt++;
                        if (g_sys_context.g_motor_status[i].stall_cnt >= SAFETY_STALL_MAX_CNT) {
                            has_stall = true;
                            Debug_Printf("[ERR] Total Tune OverCurrent Stall! Motor %d (Curr=%.2fA > Limit=%.2fA). Emergency Stop (No Rebound)!\r\n",
                                         i + 1, (float)g_sys_context.g_motor_status[i].current_deciA / 100.0f,
                                         (float)app_data.stall_current_threshold / 100.0f);
                            break;
                        }
                    } else {
                        if (g_sys_context.g_motor_status[i].stall_cnt > 0) {
                            g_sys_context.g_motor_status[i].stall_cnt--;
                        }
                    }
                }
                if (has_stall) {
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg       = {CMD_STOP, active_mask, 0};
                    g_sys_context.system_fault_code = FAULT_CODE_STALL; // 置 Err1 过流堵转
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    break;
                }

                // 防线三：驱动器本体报警状态字检查
                bool has_driver_alarm = false;
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    if (g_sys_context.g_motor_status[i].driver_status_word == 0x0004) {
                        has_driver_alarm = true;
                        Debug_Printf("[ERR] Total Tune Driver Alarm! Motor %d: FaultCode=%d\r\n",
                                     i + 1, g_sys_context.g_motor_status[i].driver_fault_code);
                        break;
                    }
                }
                if (has_driver_alarm) {
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg       = {CMD_STOP, active_mask, 0};
                    g_sys_context.system_fault_code = FAULT_CODE_DRIVER_ALARM; // 置 Err6 驱动器报警
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    break;
                }

                // 解算使能轴本次微调各自的实际物理霍尔增量 abs_counts[i]
                uint32_t abs_counts[4] = {0};
                uint32_t min_counts = 0xFFFFFFFF, max_counts = 0;
                float sum_counts        = 0.0f;
                uint8_t tune_active_cnt = 0;
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    tune_active_cnt++;
                    uint32_t now_h   = g_sys_context.g_motor_status[i].hall_value;
                    uint32_t start_h = g_sys_context.g_motor_status[i].start_drive_hall;
                    int32_t diff     = (int32_t)(now_h - start_h);
                    abs_counts[i]    = (diff >= 0) ? (uint32_t)diff : (uint32_t)(-diff);

                    sum_counts += (float)abs_counts[i];
                    if (abs_counts[i] < min_counts) min_counts = abs_counts[i];
                    if (abs_counts[i] > max_counts) max_counts = abs_counts[i];
                }
                float avg_tune_counts     = (tune_active_cnt > 0) ? (sum_counts / (float)tune_active_cnt) : 0.0f;
                uint32_t tune_diff_counts = (tune_active_cnt > 0) ? (max_counts - min_counts) : 0; // 使能轴本次微调位移极差

                // 防线四：“本次微调位移极差”过大防卡扭保护 (若某轴卡死、位移差超过 1.5mm 且超过目标步长 60% 则急停)
                uint32_t tune_diff_limit = (uint32_t)roundf(g_sys_context.counts_per_mm * 1.5f);
                if (tune_diff_limit < (g_sys_context.total_tune_target_counts * 6 / 10)) {
                    tune_diff_limit = g_sys_context.total_tune_target_counts * 6 / 10;
                }
                if (tune_diff_counts > tune_diff_limit) {
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg       = {CMD_STOP, active_mask, 0};
                    g_sys_context.system_fault_code = FAULT_CODE_SYNC; // 置 Err3 同步差过大
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[ERR] Total Tune Delta Diff Exceeded! (Diff=%d > Limit=%d counts). Emergency Stop!\r\n",
                                 tune_diff_counts, tune_diff_limit);
                    break;
                }

                // 3. 逐轴独立定长步进到达判定与单轴精准刹停
                bool all_tune_done = true;
                s_total_tune_timeout_cnt++;

                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    if (!s_total_tune_axis_done[i]) {
                        if (abs_counts[i] >= g_sys_context.total_tune_target_counts) {
                            s_total_tune_axis_done[i] = true;
                            // 该单轴已精确走满设定目标步长，立即向该单轴独立下发 CMD_STOP 刹车刹停！
                            g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                            g_sys_context.g_motor_status[i].target_speed = 0;
                            last_sent_speed[i]                           = 0;

                            Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, (uint8_t)(1 << i), 0};
                            xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                            Debug_Printf("[SYS] Total Tune Axis %d Reached Target (%d >= %d counts), Braked!\r\n",
                                         i + 1, abs_counts[i], g_sys_context.total_tune_target_counts);
                        } else {
                            all_tune_done = false;
                        }
                    }
                }

                // 4. 当使能柱子各自全部达到设定目标脉冲数，或超过 5 秒安全防卡死超时，切入停稳检测与 Flash 归档
                if (all_tune_done || s_total_tune_timeout_cnt >= 250) {
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, active_mask, 0};
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[SYS] Total Tune Step Completed! AllDone=%d, TimeoutCnt=%d. Entering Settle...\r\n",
                                 all_tune_done, s_total_tune_timeout_cnt);
                    break;
                }

                // 5. 增量平行 PID 纠偏计算 (仅在未达标的使能运动轴之间进行 ±15% 温和限幅调速，确保台面平行平移)
                int16_t base_v          = g_sys_context.base_speed;
                int16_t max_tune_adjust = (int16_t)((float)base_v * 0.15f); // 最大允许调速 ±15%
                if (max_tune_adjust < 20) max_tune_adjust = 20;

                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    if (s_total_tune_axis_done[i]) continue;

                    // 误差量：平均增量 - 本轴增量 (若落后 error > 0 需要加速，超前 error < 0 需要减速)
                    float error = avg_tune_counts - (float)abs_counts[i];
                    float adj_v = error * 1.5f; // P 控制增益，单位脉冲误差调节 1.5 RPM

                    if (adj_v > (float)max_tune_adjust) adj_v = (float)max_tune_adjust;
                    if (adj_v < -(float)max_tune_adjust) adj_v = -(float)max_tune_adjust;

                    int16_t target_v = base_v + (int16_t)adj_v;
                    if (target_v < 100) target_v = 100; // 保底 100 RPM
                    g_sys_context.g_motor_status[i].target_speed = target_v;

                    // 转速变动门限下发
                    int16_t diff_v = target_v - last_sent_speed[i];
                    if (diff_v < 0) diff_v = -diff_v;
                    if (diff_v >= 4 || last_sent_speed[i] == -1) {
                        Motor_Ctrl_Msg_t speed_msg = {CMD_SET_SPEED, (uint8_t)(1 << i), target_v};
                        if (xQueueSend(g_motor_ctrl_queue, &speed_msg, 0) == pdTRUE) {
                            last_sent_speed[i] = target_v;
                        }
                    }
                }

                // 6. 每 20ms 实时推送上位机数据
                APP_Control_DebugPrint();
                break;
            }

            // === 核心运行调速阶段（定频 20ms 数据驱动 PID 泵 + 安防防线） ===
            case SYS_STEP_TOTAL_RUNNING: {
                // 0. 采样屏障锁存等待 (Barrier Alignment)：若某轴霍尔恰好落后了微微秒 (正在接收 ACK)，让出 1ms 等其合流
                static uint32_t last_hall_seq[4] = {0};
                bool need_wait                   = false;
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    if (g_sys_context.hall_update_seq[i] == last_hall_seq[i] && modbus_masters[i].state != MODBUS_STATE_IDLE) {
                        need_wait = true;
                        break;
                    }
                }
                if (need_wait) {
                    vTaskDelay(pdMS_TO_TICKS(1));
                    MID_Modbus_Process_1ms();
                }
                for (int i = 0; i < 4; i++) {
                    last_hall_seq[i] = g_sys_context.hall_update_seq[i];
                }

                // 1. 无条件优先从内存缓存解算 4 轴最新绝对高度 + 统一计算位移增量 ΔH 与绝对伸出行程 travel_rel 及其统计量
                APP_Control_UpdateStateAndStatistics();

                // 校验运行过程中的行程上下限到界判定 (到达边界自动软停机，针对当前使能轴)
                bool is_running_forward = false;
                bool is_running_reverse = false;
                for (int i = 0; i < 4; i++) {
                    if (active_mask & (1 << i)) {
                        if (g_sys_context.g_motor_status[i].target_cmd == CMD_FORWARD) is_running_forward = true;
                        if (g_sys_context.g_motor_status[i].target_cmd == CMD_REVERSE) is_running_reverse = true;
                        break;
                    }
                }
                bool reach_limit = false;

                if (is_running_forward) {
                    for (int i = 0; i < 4; i++) {
                        if (!(active_mask & (1 << i))) continue;
                        if (g_sys_context.travel_rel[i] >= (float)g_sys_context.max_travel_hall) {
                            reach_limit = true;
                            Debug_Printf("[SYS] Auto Stop: Motor %d reached max travel limit (%.1f >= %d)\r\n",
                                         i + 1, g_sys_context.travel_rel[i], g_sys_context.max_travel_hall);
                            break;
                        }
                    }
                } else if (is_running_reverse) {
                    // 反向运动触及底部零点自动软停机
                    for (int i = 0; i < 4; i++) {
                        if (!(active_mask & (1 << i))) continue;
                        if (g_sys_context.travel_rel[i] <= 0.0f) {
                            reach_limit = true;
                            Debug_Printf("[SYS] Auto Stop: Motor %d reached zero bottom limit (%.1f <= 0)\r\n",
                                         i + 1, g_sys_context.travel_rel[i]);
                            break;
                        }
                    }
                }

                if (has_event) {
                    if ((mb_src == SYS_MOTION_SRC_SIGNAL && sig_msg.signal_id == MID_SIGNAL_REMOT_2) ||
                        (mb_src == SYS_MOTION_SRC_KEY)) {
                        // C 键 (REMOT_2) 或物理面板按键打断：专用停止按键，触发 CMD_STOP 停机！
                        xQueueReset(g_motor_ctrl_queue);

                        for (int i = 0; i < 4; i++) {
                            g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                            g_sys_context.g_motor_status[i].target_speed = 0;
                            last_sent_speed[i]                           = 0;
                        }

                        Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, active_mask, 0};
                        APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                        xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                        Debug_Printf("[SYS] Motion Interrupted (Remot C or Panel Key), sending CMD_STOP...\r\n");
                    }
                } else if (reach_limit) {
                    // 触及行程上限或零点到界停机
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }

                    Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, active_mask, 0};
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                } else if (APP_Control_CheckSafety()) {
                    xQueueReset(g_motor_ctrl_queue);
                    if (g_sys_context.system_fault_code == FAULT_CODE_STALL) {
                        uint8_t current_cmd = CMD_STOP;
                        for (int i = 0; i < 4; i++) {
                            if (active_mask & (1 << i)) {
                                current_cmd = g_sys_context.g_motor_status[i].target_cmd;
                                break;
                            }
                        }

                        // 【安全脱困策略分流】：
                        // 1. 仅当使能轴处于“同步下降 (CMD_REVERSE)”过程中过流，为了防止台面压人/卡物，触发“过流反弹”防夹脱困
                        if (current_cmd == CMD_REVERSE) {
                            g_sys_context.system_fault_code     = FAULT_CODE_PINCH; // 下降防夹反弹专属故障码 (Err4)
                            g_sys_context.rebound_cmd           = CMD_FORWARD;      // 向上反弹退避
                            g_sys_context.rebound_target_counts = (uint32_t)roundf((float)app_data.rebound_travel_mm * g_sys_context.counts_per_mm);
                            g_sys_context.active_motor_mask     = active_mask;

                            // 【安全制动第一步】：立即使能轴下发 CMD_STOP (0x0009) 刹车减速，消除下降惯性，防反接冲击
                            for (int i = 0; i < 4; i++) {
                                g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                                g_sys_context.g_motor_status[i].target_speed = 0;
                                last_sent_speed[i]                           = 0;
                            }

                            Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, active_mask, 0};
                            MID_Brake_Release(); // 保持机械抱闸释放，准备反弹
                            xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));

                            g_sys_context.system_step = SYS_STEP_TOTAL_REBOUND;
                            Debug_Printf("[SYS] Downward Pinch Stall! Step 1: Emergency Braking Cushion (80ms) for Rebound...\r\n");
                        } else {
                            // 2. “同步上升 (CMD_FORWARD)”过程中过流：台面顶阻/超重，绝不反弹！立即刹车急停进入 FAULT_STOP 锁死 (Err1)
                            g_sys_context.system_fault_code = FAULT_CODE_STALL;
                            for (int i = 0; i < 4; i++) {
                                g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                                g_sys_context.g_motor_status[i].target_speed = 0;
                                last_sent_speed[i]                           = 0;
                            }
                            Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, active_mask, 0};
                            APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                            xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                            Debug_Printf("[ERR] Upward OverCurrent Stall Detected! Immediate Emergency Stop & Lockout (Err1)...\r\n");
                        }
                    } else {
                        // 通信中断/同步差超限：先下发使能轴 STOP 切入 SYS_STEP_TOTAL_DONE 进行刹车停稳与绝对位置归档！
                        bool has_comm_err = false;
                        for (int i = 0; i < 4; i++) {
                            if (!(active_mask & (1 << i))) continue;
                            if (g_sys_context.g_motor_status[i].comm_error > 0) {
                                has_comm_err = true;
                                break;
                            }
                        }
                        if (has_comm_err) {
                            g_sys_context.system_fault_code = FAULT_CODE_COMM; // 标记 485 通信故障
                            Debug_Printf("[SYS] Safety Protection: 485 Comm Error Detected! Stopping & Archiving...\r\n");
                        } else {
                            g_sys_context.system_fault_code = FAULT_CODE_SYNC; // 标记严重同步差故障
                            Debug_Printf("[SYS] Safety Protection: Sync Diff Exceeded! Stopping & Archiving...\r\n");
                        }

                        for (int i = 0; i < 4; i++) {
                            g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                            g_sys_context.g_motor_status[i].target_speed = 0;
                            last_sent_speed[i]                           = 0;
                        }
                        Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, active_mask, 0};
                        APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                        xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    }
                } else {
                    // 2. 1000ms 直线斜坡缓启动 (RAMP_UP_TOTAL_STEPS 帧 x 20ms = 1000ms，起点 RAMP_UP_START_RPM 直线斜坡升速)
                    int16_t run_base_speed = g_sys_context.base_speed;
                    if (g_sys_context.ramp_cnt < RAMP_UP_TOTAL_STEPS) {
                        g_sys_context.ramp_cnt++;
                        int16_t start_rpm = RAMP_UP_START_RPM;
                        if (run_base_speed > start_rpm) {
                            run_base_speed = start_rpm + (int16_t)((int32_t)(run_base_speed - start_rpm) * g_sys_context.ramp_cnt / RAMP_UP_TOTAL_STEPS);
                        }
                    }

                    // 3. 极速 PID 位置同步计算 (动态限幅 + 防饱和速度平移，常规长行程联动运行生效)
                    APP_Control_RunPID(run_base_speed);

                    // 4. 【按需门限下发】转速变动幅度 >= 2 RPM 时才向队列推送写速度命令 (防抖且绝对不压制 PID)
                    for (int i = 0; i < 4; i++) {
                        if (!(active_mask & (1 << i))) continue;
                        int16_t diff_v = g_sys_context.g_motor_status[i].target_speed - last_sent_speed[i];
                        if (diff_v < 0) diff_v = -diff_v;

                        if (diff_v >= 2 || last_sent_speed[i] == -1) {
                            Motor_Ctrl_Msg_t speed_msg;
                            speed_msg.cmd_type   = CMD_SET_SPEED;
                            speed_msg.motor_mask = (1 << i);
                            speed_msg.speed_rpm  = g_sys_context.g_motor_status[i].target_speed;

                            if (xQueueSend(g_motor_ctrl_queue, &speed_msg, 0) == pdTRUE) {
                                last_sent_speed[i] = g_sys_context.g_motor_status[i].target_speed;
                            }
                        }
                    }

                    // 5. 每 20ms 实时推送上位机数据 (50Hz 零抖动连续平滑数据流)
                    APP_Control_DebugPrint();
                }
                break;
            }

            // === SYS_STEP_TOTAL_REBOUND 阶段：堵转后整体向反方向反弹运行中 ===
            case SYS_STEP_TOTAL_REBOUND: {
                static uint8_t s_rebound_brake_ticks = 4; // 4 帧 x 20ms = 80ms 刹车减速缓冲
                static bool s_rebound_active         = false;

                if (!s_rebound_active) {
                    s_rebound_active      = true;
                    s_rebound_brake_ticks = 4; // 80ms 刹停缓冲
                    Debug_Printf("[SYS] TOTAL_REBOUND: Entering 80ms Braking Cushion...\r\n");
                }

                // 1. 无条件调用通用解算函数更新 4 轴绝对高度、伸出行程及极差
                APP_Control_UpdateStateAndStatistics();

                // 2. 安防防线 1：485 通信中断检查
                bool has_comm_err = false;
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    if (g_sys_context.g_motor_status[i].comm_error >= SAFETY_COMM_ERR_MAX_CNT) {
                        has_comm_err = true;
                        Debug_Printf("[ERR] Rebound Comm Loss! Motor %d (CommErr=%d >= Limit=%d)\r\n",
                                     i + 1, g_sys_context.g_motor_status[i].comm_error, SAFETY_COMM_ERR_MAX_CNT);
                        break;
                    }
                }
                if (has_comm_err) {
                    s_rebound_active = false;
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg       = {CMD_STOP, active_mask, 0};
                    g_sys_context.system_fault_code = FAULT_CODE_COMM;
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[SYS] Rebound Aborted: Comm Loss! Entering TOTAL_DONE -> Lockout Err2.\r\n");
                    break;
                }

                // 3. 刹车减速缓冲期处理：电机刹停减速至 0，消除反接与电气冲击
                if (s_rebound_brake_ticks > 0) {
                    s_rebound_brake_ticks--;
                    if (s_rebound_brake_ticks == 0) {
                        // 刹车缓冲结束：此时电机已完全静止！刷新真实霍尔零点基准，正式启动向上反弹
                        uint16_t rebound_speed_rpm = g_sys_context.calc_base_rpm / 2;
                        if (rebound_speed_rpm < 100) rebound_speed_rpm = 100;

                        for (int i = 0; i < 4; i++) {
                            g_sys_context.rebound_start_hall[i]              = g_sys_context.g_motor_status[i].hall_value;
                            g_sys_context.g_motor_status[i].start_drive_hall = g_sys_context.g_motor_status[i].hall_value;
                            g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                            g_sys_context.g_motor_status[i].target_cmd       = g_sys_context.rebound_cmd;
                            g_sys_context.g_motor_status[i].target_speed     = rebound_speed_rpm;
                            g_sys_context.g_motor_status[i].stall_cnt        = 0;
                            last_sent_speed[i]                               = -1;
                        }

                        Motor_Ctrl_Msg_t speed_msg = {CMD_SET_SPEED, active_mask, rebound_speed_rpm};
                        Motor_Ctrl_Msg_t cmd_msg   = {(Motor_Cmd_Type_t)g_sys_context.rebound_cmd, active_mask, 0};

                        MID_Brake_Release(); // 453 机械抱闸：确保抱闸处于释放状态

                        xQueueSend(g_motor_ctrl_queue, &speed_msg, pdMS_TO_TICKS(10));
                        xQueueSend(g_motor_ctrl_queue, &cmd_msg, pdMS_TO_TICKS(10));

                        Debug_Printf("[SYS] Braking Buffer Complete! Starting Upward Rebound: Speed=%dRPM, Distance=%dmm (TargetCounts=%d)...\r\n",
                                     rebound_speed_rpm, app_data.rebound_travel_mm, g_sys_context.rebound_target_counts);
                    }
                    APP_Control_DebugPrint();
                    break; // 缓冲期间不进行 PID 纠偏及反弹距离累加，等待刹停完成
                }

                // 4. 安防防线 2：反弹过程中使能轴同步差超限（防止个别柱子卡死不退导致台面严重倾斜拉偏）
                if (g_sys_context.max_travel_diff > (float)g_sys_context.max_sync_diff_hall) {
                    s_rebound_active = false;
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg       = {CMD_STOP, active_mask, 0};
                    g_sys_context.system_fault_code = FAULT_CODE_SYNC; // 统一归入同步差超限 (Err3)
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[ERR] Rebound Sync Diff Exceeded (Diff=%.1f > Limit=%d)! Stopping Rebound & Entering SYS_STEP_TOTAL_DONE to Lockout Err3...\r\n",
                                 g_sys_context.max_travel_diff, g_sys_context.max_sync_diff_hall);
                    break;
                }

                // 4. 安防防线 3：反弹过程中过流检查 (反弹上升受阻)
                bool has_rebound_stall = false;
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    if (g_sys_context.g_motor_status[i].current_deciA > app_data.stall_current_threshold) {
                        g_sys_context.g_motor_status[i].stall_cnt++;
                        if (g_sys_context.g_motor_status[i].stall_cnt >= SAFETY_STALL_MAX_CNT) {
                            has_rebound_stall = true;
                            Debug_Printf("[ERR] Rebound OverCurrent Stall! Motor %d (Curr=%.2fA > Limit=%.2fA)\r\n",
                                         i + 1, (float)g_sys_context.g_motor_status[i].current_deciA / 100.0f,
                                         (float)app_data.stall_current_threshold / 100.0f);
                            break;
                        }
                    } else {
                        if (g_sys_context.g_motor_status[i].stall_cnt > 0) {
                            g_sys_context.g_motor_status[i].stall_cnt--;
                        }
                    }
                }
                if (has_rebound_stall) {
                    s_rebound_active = false;
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg       = {CMD_STOP, active_mask, 0};
                    g_sys_context.system_fault_code = FAULT_CODE_STALL; // 统一归入过流堵转 (Err1)
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[SYS] Rebound Stall Detected! Stopping Rebound & Entering SYS_STEP_TOTAL_DONE to Lockout Err1...\r\n");
                    break;
                }

                // 5. 解算使能轴反弹累计增量 (计算平均反弹步数)
                float sum_reb_counts = 0.0f;
                uint8_t reb_cnt      = 0;
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    reb_cnt++;
                    int32_t reb_h = g_sys_context.g_motor_status[i].current_abs_hall - g_sys_context.g_motor_status[i].base_abs_hall;
                    sum_reb_counts += (reb_h >= 0) ? (float)reb_h : (float)(-reb_h);
                }
                float avg_reb_counts = (reb_cnt > 0) ? (sum_reb_counts / (float)reb_cnt) : 0.0f;

                // 6. 堵转反弹阶段实时 PID 纠偏调速 (保持顶部台面绝对平行，采用正常速度的一半)
                uint16_t rebound_speed_rpm = g_sys_context.calc_base_rpm / 2;
                if (rebound_speed_rpm < 100) rebound_speed_rpm = 100;
                APP_Control_RunPID(rebound_speed_rpm);

                // 【按需门限下发】向使能轴下发 PID 平行纠偏调整速度
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    int16_t diff_v = g_sys_context.g_motor_status[i].target_speed - last_sent_speed[i];
                    if (diff_v < 0) diff_v = -diff_v;

                    if (diff_v >= 2 || last_sent_speed[i] == -1) {
                        Motor_Ctrl_Msg_t speed_msg = {CMD_SET_SPEED, (uint8_t)(1 << i), g_sys_context.g_motor_status[i].target_speed};
                        Motor_Ctrl_Msg_t cmd_msg   = {(Motor_Cmd_Type_t)g_sys_context.rebound_cmd, (uint8_t)(1 << i), 0};

                        if (xQueueSend(g_motor_ctrl_queue, &speed_msg, 0) == pdTRUE) {
                            last_sent_speed[i] = g_sys_context.g_motor_status[i].target_speed;
                        }
                        xQueueSend(g_motor_ctrl_queue, &cmd_msg, 0);
                    }
                }

                // 7. 检查反弹终止条件（完成目标行程，到达到界物理极值，或用户中途按下 C 键/停止按键）
                bool rebound_done = (avg_reb_counts >= (float)g_sys_context.rebound_target_counts);

                if (has_event && mb_src == SYS_MOTION_SRC_SIGNAL && sig_msg.event == MID_SIGNAL_EVT_TRIGGER) {
                    if (sig_msg.signal_id == MID_SIGNAL_REMOT_2) {
                        // C 键 (REMOT_2)：专用停止按键，手动打断反弹流程！
                        rebound_done = true;
                        Debug_Printf("[SYS] Rebound Interrupted by User Remot C Key!\r\n");
                    }
                }

                if (!rebound_done) {
                    if (g_sys_context.rebound_cmd == CMD_REVERSE) {
                        for (int i = 0; i < 4; i++) {
                            if (!(active_mask & (1 << i))) continue;
                            if (g_sys_context.travel_rel[i] <= 0.0f) {
                                rebound_done = true;
                                break;
                            }
                        }
                    } else if (g_sys_context.rebound_cmd == CMD_FORWARD) {
                        for (int i = 0; i < 4; i++) {
                            if (!(active_mask & (1 << i))) continue;
                            if (g_sys_context.travel_rel[i] >= (float)g_sys_context.max_travel_hall) {
                                rebound_done = true;
                                break;
                            }
                        }
                    }
                }

                // 8. 反弹完成：直接下发使能轴 STOP，切入 SYS_STEP_TOTAL_DONE 统一接管停稳判定与 Flash 固化！
                if (rebound_done) {
                    s_rebound_active = false;
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }

                    Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, active_mask, 0};
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[SYS] Rebound Distance Reached (AvgCounts=%.0f), Sending CMD_STOP & Entering SYS_STEP_TOTAL_DONE...\r\n", avg_reb_counts);
                }

                APP_Control_DebugPrint();
                break;
            }

            // === 停机归档阶段（逐轴独立停稳检测与停机 + 运动轴全停后单次 Flash 归档） ===
            case SYS_STEP_TOTAL_DONE:
            case SYS_STEP_TUNE_DONE: {
                // 1. 无条件调用通用解算函数更新 4 轴绝对高度、伸出行程及极差
                APP_Control_UpdateStateAndStatistics();

                bool all_active_settled = true;

                // 2. 遍历 4 个轴，进行逐轴独立静止判定
                for (int i = 0; i < 4; i++) {
                    // 若该轴未参与本次运动，无需等待其静止，直接视为已停稳
                    if (!(g_sys_context.active_motor_mask & (1 << i))) {
                        s_axis_is_settled[i] = true;
                        continue;
                    }

                    // 检查霍尔值是否与上一次采样相同
                    if (g_sys_context.g_motor_status[i].hall_value == s_last_check_halls[i]) {
                        if (!s_axis_is_settled[i]) {
                            s_axis_stable_cnt[i]++;
                            if (s_axis_stable_cnt[i] >= STOP_STABLE_CHECK_CNT) {
                                s_axis_is_settled[i] = true;

                                // 某轴一停稳，立即先对该单轴下发 0x0005 停机指令（释放刹车进入待机）
                                Motor_Ctrl_Msg_t idle_stop_msg = {CMD_IDLE_STOP, (uint8_t)(1 << i), 0};
                                xQueueSend(g_motor_ctrl_queue, &idle_stop_msg, pdMS_TO_TICKS(10));
                                Debug_Printf("[SYS] Motor %d Settled & Stable: Sent 0x0005 CMD_IDLE_STOP.\r\n", i + 1);

                                // 若该轴为单轴微调目标轴，在彻底停稳后根据真实最终脉冲精确结算 min_mount_halls
                                if (g_sys_context.is_single_tuning && i == g_sys_context.single_tune_motor_idx) {
                                    g_sys_context.is_single_tuning = false; // 消费微调标记
                                    uint32_t final_hall            = g_sys_context.g_motor_status[i].hall_value;
                                    int32_t raw_diff               = (int32_t)(final_hall - g_sys_context.single_tune_start_hall);
                                    uint32_t abs_pulse             = (raw_diff >= 0) ? (uint32_t)raw_diff : (uint32_t)(-raw_diff);

                                    int32_t tune_delta = (g_sys_context.single_tune_dir == 0) ? (int32_t)abs_pulse : -(int32_t)abs_pulse;
                                    app_data.min_mount_halls[i] += tune_delta;
                                    Debug_Printf("[SYS] Single Tune Final Settled! Motor %d (Dir=%s) FinalDelta=%d, New min_mount_hall=%d.\r\n",
                                                 i + 1, (g_sys_context.single_tune_dir == 0) ? "UP" : "DOWN",
                                                 tune_delta, app_data.min_mount_halls[i]);
                                } else if (g_sys_context.is_total_tuning && g_sys_context.system_fault_code == FAULT_CODE_NONE) {
                                    // 四柱一键微调停稳后（且无故障报错），将各轴真实物理位移增量精准计入起点高度 (调平零点)
                                    uint32_t final_hall = g_sys_context.g_motor_status[i].hall_value;
                                    int32_t raw_diff    = (int32_t)(final_hall - g_sys_context.g_motor_status[i].start_drive_hall);
                                    uint32_t abs_pulse  = (raw_diff >= 0) ? (uint32_t)raw_diff : (uint32_t)(-raw_diff);

                                    bool is_up         = (g_sys_context.g_motor_status[i].last_motion_cmd == CMD_FORWARD);
                                    int32_t tune_delta = is_up ? (int32_t)abs_pulse : -(int32_t)abs_pulse;
                                    app_data.min_mount_halls[i] += tune_delta;
                                    Debug_Printf("[SYS] Total Tune Axis Settled! Motor %d (Dir=%s) FinalDelta=%d, New min_mount_hall=%d.\r\n",
                                                 i + 1, is_up ? "UP" : "DOWN",
                                                 tune_delta, app_data.min_mount_halls[i]);
                                }

                                // 刷新该轴在内存中的绝对高度和基准
                                app_data.motor_abs_halls[i]                      = g_sys_context.g_motor_status[i].current_abs_hall;
                                g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                                g_sys_context.g_motor_status[i].start_drive_hall = g_sys_context.g_motor_status[i].hall_value;
                                g_sys_context.g_motor_status[i].stall_cnt        = 0;
                                g_sys_context.delta_h[i]                         = 0.0f;
                            }
                        }
                    } else {
                        // 霍尔值仍在变动（尚未停稳），更新历史并清零该轴稳定计数
                        s_last_check_halls[i] = g_sys_context.g_motor_status[i].hall_value;
                        s_axis_stable_cnt[i]  = 0;
                        s_axis_is_settled[i]  = false;
                    }

                    if (!s_axis_is_settled[i]) {
                        all_active_settled = false;
                    }
                }

                // 3. 当所有参与运动的轴均已彻底停稳（单轴微调仅需 1 轴，四轴联动需 4 轴）：
                if (all_active_settled) {
                    // 重置停稳检测历史
                    for (int i = 0; i < 4; i++) {
                        s_last_check_halls[i] = 0xFFFFFFFF;
                        s_axis_stable_cnt[i]  = 0;
                        s_axis_is_settled[i]  = false;
                    }

                    APP_Data_Storage();
                    MID_Brake_Lock(); // 453 机械抱闸安全时序：运动轴彻底停稳后断电抱紧自锁
                    Debug_Printf("[SYS] All Active Motors Settled & Flash Archived Successfully!\r\n");

                    // 若此前正在执行四柱同步微调，清除微调标志并在无故障时触发提示音
                    if (g_sys_context.is_total_tuning) {
                        g_sys_context.is_total_tuning = false;
                        if (g_sys_context.system_fault_code == FAULT_CODE_NONE) {
                            MID_Buzzer_TriggerBeep(40);
                        }
                    }

                    // 以最新的 min_mount_halls 重新计算 4 轴绝对伸出行程 travel_rel 及极差 max_travel_diff
                    APP_Control_UpdateStateAndStatistics();

                    // 重置 4 轴 PID 控制器历史状态
                    for (int i = 0; i < 4; i++) {
                        APP_PID_Init(&motor_pids[i], PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD,
                                     PID_DEFAULT_DEADZONE, PID_DEFAULT_OUT_MAX, PID_DEFAULT_OUT_MIN, PID_DEFAULT_IOUT_MAX);
                    }

                    g_sys_context.base_speed = 0;
                    APP_Control_DebugPrint();

                    // 4. 停稳归档完成后的流转决策 (异常安防锁死 / 自愈对齐 / 正常就绪)
                    bool has_comm_fault = false;
                    for (int i = 0; i < 4; i++) {
                        if (!(active_mask & (1 << i))) continue;
                        if (g_sys_context.g_motor_status[i].comm_error > 0) {
                            has_comm_fault = true;
                            break;
                        }
                    }

                    bool has_driver_fault = false;
                    for (int i = 0; i < 4; i++) {
                        if (!(active_mask & (1 << i))) continue;
                        if (g_sys_context.g_motor_status[i].driver_status_word == 0x0004) {
                            has_driver_fault = true;
                            break;
                        }
                    }

                    // A. 通信故障检查
                    if (has_comm_fault || g_sys_context.system_fault_code == FAULT_CODE_COMM) {
                        g_sys_context.system_fault_code = FAULT_CODE_COMM;
                        g_sys_context.system_step       = SYS_STEP_FAULT_STOP;
                        Sys_Mode_Set(SYS_MODE_FAULT_LOCKED);
                        MID_Brake_Lock(); // 致命通信故障抱死自锁
                        Debug_Printf("[SYS] Stop Check: 485 Comm Fault Active! Entering SYS_STEP_FAULT_STOP Lockout (Err2).\r\n");
                    }
                    // B. 驱动器本体故障检查
                    else if (has_driver_fault || g_sys_context.system_fault_code == FAULT_CODE_DRIVER_ALARM) {
                        g_sys_context.system_fault_code = FAULT_CODE_DRIVER_ALARM;
                        g_sys_context.system_step       = SYS_STEP_FAULT_STOP;
                        Sys_Mode_Set(SYS_MODE_FAULT_LOCKED);
                        MID_Brake_Lock(); // 驱动器报警抱死自锁
                        Debug_Printf("[SYS] Stop Check: Driver Alarm Active! Entering SYS_STEP_FAULT_STOP Lockout (Err6).\r\n");
                    }
                    // C. 过流堵转故障检查 (包括：上升堵转、微调堵转、反弹中再过流、调平堵转)
                    else if (g_sys_context.system_fault_code == FAULT_CODE_STALL) {
                        g_sys_context.system_fault_code = FAULT_CODE_STALL;
                        g_sys_context.system_step       = SYS_STEP_FAULT_STOP;
                        Sys_Mode_Set(SYS_MODE_FAULT_LOCKED);
                        MID_Brake_Lock(); // 过流堵转抱死自锁
                        Debug_Printf("[SYS] Stop Check: OverCurrent Stall Active! Entering SYS_STEP_FAULT_STOP Lockout (Err1).\r\n");
                    }
                    // D. 下降防夹反弹停机检查 (退避 30mm 结束停稳归档)
                    else if (g_sys_context.system_fault_code == FAULT_CODE_PINCH) {
                        g_sys_context.system_fault_code = FAULT_CODE_PINCH;
                        g_sys_context.system_step       = SYS_STEP_FAULT_STOP;
                        Sys_Mode_Set(SYS_MODE_FAULT_LOCKED);
                        MID_Brake_Lock(); // 防夹反弹完成抱死自锁
                        Debug_Printf("[SYS] Stop Check: Downward Pinch Rebound Done! Entering SYS_STEP_FAULT_STOP Lockout (Err4).\r\n");
                    }
                    // D. 同步故障检查 (包括：反弹同步差超限、调平超时/发散、运行同步差严重超限)
                    else if (g_sys_context.system_fault_code == FAULT_CODE_SYNC) {
                        g_sys_context.system_fault_code = FAULT_CODE_SYNC;
                        g_sys_context.system_step       = SYS_STEP_FAULT_STOP;
                        Sys_Mode_Set(SYS_MODE_FAULT_LOCKED);
                        MID_Brake_Lock(); // 同步严重超限抱死自锁
                        Debug_Printf("[SYS] Stop Check: Sync Diff Fault Active! Entering SYS_STEP_FAULT_STOP Lockout (Err3).\r\n");
                    }
                    // E. 无任何致命故障，仅台面极差超标 -> 触发自主重平自愈
                    else if (g_sys_context.max_travel_diff > (float)g_sys_context.max_sync_diff_hall) {
                        for (int i = 0; i < 4; i++) {
                            g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                            g_sys_context.g_motor_status[i].start_drive_hall = g_sys_context.g_motor_status[i].hall_value;
                        }
                        g_sys_context.active_motor_mask = active_mask;
                        g_sys_context.system_step       = SYS_STEP_AUTO_ALIGN;
                        Sys_Mode_Set(SYS_MODE_AUTO_ALIGN);
                        MID_Brake_Release();           // 453 抱闸控制：启动自愈重平前通电松开抱闸
                        vTaskDelay(pdMS_TO_TICKS(80)); // 硬件脱开延时 80ms
                        Debug_Printf("[SYS] Stop Check: Table Sync Diff (Diff=%.1f > Limit=%d) with No Faults! Triggering AUTO_ALIGN Self-Healing...\r\n",
                                     g_sys_context.max_travel_diff, g_sys_context.max_sync_diff_hall);
                    }
                    // F. 无任何故障且台面平整 -> 正常就绪待命
                    else {
                        g_sys_context.system_fault_code = FAULT_CODE_NONE;
                        g_sys_context.system_step       = SYS_STEP_READY;
                        extern uint8_t dim1;
                        if (!Sys_Mode_IsMenuActive() && dim1 == 0) {
                            Sys_Mode_Set(SYS_MODE_STANDBY);
                        }
                        Debug_Printf("[SYS] State -> READY (AbsHalls:[%d,%d,%d,%d], MaxDiff=%.1f)\r\n",
                                     g_sys_context.g_motor_status[0].current_abs_hall,
                                     g_sys_context.g_motor_status[1].current_abs_hall,
                                     g_sys_context.g_motor_status[2].current_abs_hall,
                                     g_sys_context.g_motor_status[3].current_abs_hall,
                                     g_sys_context.max_travel_diff);
                        Debug_Printf("[SYS] Flash Data (SavedAbs:[%d,%d,%d,%d], MinMount:[%d,%d,%d,%d])\r\n",
                                     app_data.motor_abs_halls[0], app_data.motor_abs_halls[1],
                                     app_data.motor_abs_halls[2], app_data.motor_abs_halls[3],
                                     app_data.min_mount_halls[0], app_data.min_mount_halls[1],
                                     app_data.min_mount_halls[2], app_data.min_mount_halls[3]);
                    }
                }
                break;
            }

            // === SYS_STEP_AUTO_ALIGN 阶段：四轴自主台面平行恢复 (自愈重平) ===
            case SYS_STEP_AUTO_ALIGN: {
                static bool s_align_active          = false;
                static float s_best_align_diff      = 999999.0f;
                static uint16_t s_no_progress_ticks = 0;

                // 初次进入 AUTO_ALIGN 阶段时初始化基准与看门狗
                if (!s_align_active) {
                    s_align_active      = true;
                    s_best_align_diff   = g_sys_context.max_travel_diff;
                    s_no_progress_ticks = 0;
                    Debug_Printf("[SYS] AUTO_ALIGN Started: Initial MaxDiff=%.1f\r\n", s_best_align_diff);
                }

                // 1. 实时解算 4 轴绝对高度与伸出行程 travel_rel
                APP_Control_UpdateStateAndStatistics();

                // 2. 检查用户按键打断 (如按 C 键紧急停止调平)
                if (has_event && sig_msg.event == MID_SIGNAL_EVT_TRIGGER && sig_msg.signal_id == MID_SIGNAL_REMOT_2) {
                    s_align_active = false;
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, active_mask, 0};
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TUNE_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[SYS] AUTO_ALIGN Interrupted by User Remot C Key!\r\n");
                    break;
                }

                // -------------------------------------------------------------
                // 安防防线 1：485 通信连续中断检查 (若某轴掉线，经 TOTAL_DONE 停稳落盘再锁死)
                // -------------------------------------------------------------
                bool has_comm_err = false;
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    if (g_sys_context.g_motor_status[i].comm_error >= SAFETY_COMM_ERR_MAX_CNT) {
                        has_comm_err = true;
                        Debug_Printf("[ERR] Auto-Align Comm Loss! Motor %d (CommErr=%d >= Limit=%d)\r\n",
                                     i + 1, g_sys_context.g_motor_status[i].comm_error, SAFETY_COMM_ERR_MAX_CNT);
                        break;
                    }
                }
                if (has_comm_err) {
                    s_align_active = false;
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg       = {CMD_STOP, active_mask, 0};
                    g_sys_context.system_fault_code = FAULT_CODE_COMM;
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[SYS] Auto-Align Aborted: 485 Comm Loss! Entering TOTAL_DONE -> Lockout (Err2).\r\n");
                    break;
                }

                // -------------------------------------------------------------
                // 安防防线 2：单轴过流堵转检查 (防止调平追赶时卡死顶坏机械，经 TOTAL_DONE 停稳落盘再锁死)
                // -------------------------------------------------------------
                bool has_align_stall = false;
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    if (g_sys_context.g_motor_status[i].current_deciA > app_data.stall_current_threshold) {
                        g_sys_context.g_motor_status[i].stall_cnt++;
                        if (g_sys_context.g_motor_status[i].stall_cnt >= SAFETY_STALL_MAX_CNT) {
                            has_align_stall = true;
                            Debug_Printf("[ERR] Auto-Align Stall! Motor %d (Curr=%.2fA > Limit=%.2fA)\r\n",
                                         i + 1, (float)g_sys_context.g_motor_status[i].current_deciA / 100.0f,
                                         (float)app_data.stall_current_threshold / 100.0f);
                            break;
                        }
                    } else {
                        if (g_sys_context.g_motor_status[i].stall_cnt > 0) {
                            g_sys_context.g_motor_status[i].stall_cnt--;
                        }
                    }
                }
                if (has_align_stall) {
                    s_align_active = false;
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg       = {CMD_STOP, active_mask, 0};
                    g_sys_context.system_fault_code = FAULT_CODE_STALL; // 统一归入过流堵转 (Err1)
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[SYS] Auto-Align Aborted: OverCurrent Stall! Entering TOTAL_DONE -> Lockout (Err1).\r\n");
                    break;
                }

                // -------------------------------------------------------------
                // 安防防线 3：差距未缩小时才计时，若差距在缩小则持续刷新看门狗 (6秒)；
                // 自适应发散保护：当前极差比历史最优值恶化反弹超过 1 个同步允许差 (max_sync_diff_hall)，判定为走反发散
                // -------------------------------------------------------------
                if (g_sys_context.max_travel_diff < s_best_align_diff - 2.0f) {
                    s_best_align_diff   = g_sys_context.max_travel_diff;
                    s_no_progress_ticks = 0; // 差距在实质缩小，持续刷新无进展看门狗计时器！
                } else {
                    s_no_progress_ticks++; // 差距停滞或未缩小，累计无进展时间
                }

                // 自适应发散判定：反弹恶化超过 max_sync_diff_hall 即刻刹车，天然杜绝入口死锁且全程动态紧缩
                bool is_diverged = (g_sys_context.max_travel_diff > s_best_align_diff + (float)g_sys_context.max_sync_diff_hall);

                if (s_no_progress_ticks >= AUTO_ALIGN_TIMEOUT_TICKS || is_diverged) {
                    s_align_active = false;
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg       = {CMD_STOP, active_mask, 0};
                    g_sys_context.system_fault_code = FAULT_CODE_SYNC; // 归为同步故障 (Err3)
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[ERR] Auto-Align Failed: %s (Diff=%.1f, Best=%.1f, Ticks=%d/%d)! Entering TOTAL_DONE -> Lockout (Err3).\r\n",
                                 is_diverged ? "Sync Diff Diverged (Rebound > SyncLimit)" : "No-Progress Timeout (6s)",
                                 g_sys_context.max_travel_diff, s_best_align_diff, s_no_progress_ticks, AUTO_ALIGN_TIMEOUT_TICKS);
                    break;
                }

                // 2. 检查极差收敛对齐终止条件 (收敛至 max_sync_diff_hall * AUTO_ALIGN_TARGET_DIFF_RATIO 以内)
                float target_converge_diff = (float)g_sys_context.max_sync_diff_hall * AUTO_ALIGN_TARGET_DIFF_RATIO;
                if (g_sys_context.max_travel_diff <= target_converge_diff) {
                    s_align_active = false;
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }

                    Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, active_mask, 0};
                    APP_Control_PrepareStopSettling(active_mask, SYS_STEP_TUNE_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[SYS] AUTO_ALIGN Completed! Sync Diff Converged (Diff=%.1f <= Limit=%.1f). Re-entering SYS_STEP_TUNE_DONE...\r\n",
                                 g_sys_context.max_travel_diff, target_converge_diff);
                    break;
                }

                // 3. 自主独立追赶与回压控制：以平均伸出行程 avg_travel 为目标基准线 (速度取正常计算转速的一半)
                float target_line = g_sys_context.avg_travel;
                float deadzone    = 15.0f; // 死区 15 counts (约 0.13mm)

                uint16_t align_speed_rpm = g_sys_context.calc_base_rpm / 2;
                if (align_speed_rpm < 100) align_speed_rpm = 100;

                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    float diff            = target_line - g_sys_context.travel_rel[i];
                    uint8_t desired_cmd   = CMD_STOP;
                    int16_t desired_speed = 0;

                    if (diff > deadzone) {
                        desired_cmd   = CMD_FORWARD; // 偏低轴：正转上升追赶
                        desired_speed = align_speed_rpm;
                    } else if (diff < -deadzone) {
                        desired_cmd   = CMD_REVERSE; // 偏高轴：反转下降回压
                        desired_speed = align_speed_rpm;
                    } else {
                        desired_cmd   = CMD_STOP;
                        desired_speed = 0;
                    }

                    if (g_sys_context.g_motor_status[i].target_cmd != desired_cmd ||
                        g_sys_context.g_motor_status[i].target_speed != desired_speed) {
                        g_sys_context.g_motor_status[i].target_cmd   = desired_cmd;
                        g_sys_context.g_motor_status[i].target_speed = desired_speed;
                        last_sent_speed[i]                           = desired_speed;

                        Motor_Ctrl_Msg_t speed_msg = {CMD_SET_SPEED, (uint8_t)(1 << i), desired_speed};
                        Motor_Ctrl_Msg_t cmd_msg   = {(Motor_Cmd_Type_t)desired_cmd, (uint8_t)(1 << i), 0};
                        xQueueSend(g_motor_ctrl_queue, &speed_msg, 0);
                        xQueueSend(g_motor_ctrl_queue, &cmd_msg, 0);
                    }
                }

                APP_Control_DebugPrint();
                break;
            }

            // === SYS_STEP_FAULT_STOP 阶段：致命故障急停锁死状态 (通信恢复自动发急停与复位，确认应答后再开放消警) ===
            case SYS_STEP_FAULT_STOP: {
                static Motor_ctl_Step_t s_last_step = SYS_STEP_Boot;
                static bool s_last_comm_ok          = false;
                static uint16_t s_auto_reset_timer  = 0;
                static bool s_driver_recovered      = false;
                uint8_t active_mask                 = App_Data_GetColumnMotorMask();

                // 刚切入 FAULT_STOP 状态时的初始化
                if (s_last_step != SYS_STEP_FAULT_STOP) {
                    s_last_step        = SYS_STEP_FAULT_STOP;
                    s_last_comm_ok     = false;
                    s_auto_reset_timer = 0;
                    s_driver_recovered = false;
                }

                // 1. 检查使能轴 485 通信状态
                bool comm_ok = true;
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    if (g_sys_context.g_motor_status[i].comm_error > 0) {
                        comm_ok = false;
                        break;
                    }
                }

                // 2. 检查使能轴驱动器本体是否处于故障红灯状态 (0x2100 状态字 == 0x0004)
                bool any_driver_fault = false;
                for (int i = 0; i < 4; i++) {
                    if (!(active_mask & (1 << i))) continue;
                    if (g_sys_context.g_motor_status[i].driver_status_word == 0x0004) {
                        any_driver_fault = true;
                        // 若主控当前尚未标记为特定故障或仍为通信故障，当通信恢复但驱动器故障时，显示为 Err6
                        if (comm_ok && (g_sys_context.system_fault_code == FAULT_CODE_COMM || g_sys_context.system_fault_code == FAULT_CODE_NONE)) {
                            g_sys_context.system_fault_code = FAULT_CODE_DRIVER_ALARM;
                        }
                        break;
                    }
                }

                // 若通信断开，锁定取消报警按钮，重置标志
                if (!comm_ok) {
                    s_last_comm_ok     = false;
                    s_driver_recovered = false;
                    s_auto_reset_timer = 0;
                }

                // 3. 通信重连后：先发送紧急停机和故障恢复
                if (comm_ok) {
                    // A. 通信刚恢复的第一时间：立即先发送紧急刹车停机 (0x0009) 和故障恢复 (0x0007)
                    if (!s_last_comm_ok) {
                        s_last_comm_ok     = true;
                        s_driver_recovered = false;
                        s_auto_reset_timer = 0;
                        xQueueReset(g_motor_ctrl_queue);

                        for (int i = 0; i < 4; i++) {
                            if (!(active_mask & (1 << i))) continue;
                            g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                            g_sys_context.g_motor_status[i].target_speed = 0;
                            last_sent_speed[i]                           = 0;
                        }

                        // 先发紧急刹车停机 (CMD_STOP: 0x0009)
                        Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, active_mask, 0};
                        xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));

                        // 紧接着发故障恢复 (CMD_FAULT_RESET: 0x0007)
                        Motor_Ctrl_Msg_t reset_msg = {CMD_FAULT_RESET, active_mask, 0};
                        xQueueSend(g_motor_ctrl_queue, &reset_msg, pdMS_TO_TICKS(10));

                        Debug_Printf("[SYS] 485 Comm Restored: Sent CMD_STOP (0x0009) & CMD_FAULT_RESET (0x0007). Waiting for ACK...\r\n");
                    }

                    // B. 持续监测应答：如果驱动器仍处于红灯故障中，每隔 400ms 主动重发复位 (克服驱动器自愈次数用尽)
                    if (any_driver_fault) {
                        s_driver_recovered = false;
                        if (++s_auto_reset_timer >= 20) { // 20 帧 x 20ms = 400ms
                            s_auto_reset_timer         = 0;
                            Motor_Ctrl_Msg_t reset_msg = {CMD_FAULT_RESET, active_mask, 0};
                            xQueueSend(g_motor_ctrl_queue, &reset_msg, 0);
                            Debug_Printf("[SYS] Retry CMD_FAULT_RESET (0x0007)... DriverFaults=[%d,%d,%d,%d]\r\n",
                                         g_sys_context.g_motor_status[0].driver_fault_code,
                                         g_sys_context.g_motor_status[1].driver_fault_code,
                                         g_sys_context.g_motor_status[2].driver_fault_code,
                                         g_sys_context.g_motor_status[3].driver_fault_code);
                        }
                    } else {
                        // 使能轴驱动器均已退出 0x0004 故障态，说明驱动器已确认收到复位并成功清除故障！
                        if (!s_driver_recovered) {
                            s_driver_recovered = true;
                            Debug_Printf("[SYS] Driver Reset Confirmed! All Active Motors Normal. Unlocking ClearFault Button.\r\n");
                        }
                    }
                }

                // 4. 收到应答后，再开放取消报警并自由停机的按钮
                if (has_event && (sig_msg.event == MID_SIGNAL_EVT_TRIGGER || sig_msg.event == MID_SIGNAL_EVT_LONG)) {
                    if (s_driver_recovered) {
                        if (APP_Control_ClearFault()) {
                            s_driver_recovered = false;
                            s_last_comm_ok     = false;
                            s_last_step        = SYS_STEP_TOTAL_DONE;
                        }
                    } else {
                        Debug_Printf("[SYS] Clear Fault Blocked: Waiting for Driver Reset Response (CommOk=%d, DriverFault=%d)\r\n",
                                     comm_ok, any_driver_fault);
                    }
                }
                break;
            }

            default:
                break;
        }

        // 严格 20.0ms 绝对周期挂起 (50Hz 定频调度)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}
