#include <math.h>
#include <stdlib.h>
#include "app_control.h"
#include "mid_signal.h"
#include "mid_led.h"
#include "mid_modbus.h"
#include "app_Comm.h"
#include "app_Data.h"
#include "app_pid.h"
#include "mid_Key.h"
#include "app_Menu.h"
#include "mid_brake.h"

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
    // 1. 通信连续中断检查 (连续 10 帧/40ms 接收失败触发通信保护)
    for (int i = 0; i < 4; i++) {
        if (g_sys_context.g_motor_status[i].comm_error >= SAFETY_COMM_ERR_MAX_CNT) {
            g_sys_context.system_fault_code = FAULT_CODE_COMM_ERR;
            Debug_Printf("[ERR] Safety Fault: Motor %d Comm Loss! (CommErr=%d)\r\n",
                         i, g_sys_context.g_motor_status[i].comm_error);
            return true;
        }
    }

    // 2. 轴间真实绝对高度差超限检查 (统一使用基于调平零点的绝对高度差 max_travel_diff)
    if (g_sys_context.max_travel_diff > (float)g_sys_context.max_sync_diff_hall) {
        g_sys_context.system_fault_code = FAULT_CODE_SYNC_ERR;
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
        if (g_sys_context.g_motor_status[i].current_deciA > app_data.stall_current_threshold) {
            g_sys_context.g_motor_status[i].stall_cnt++;
            if (g_sys_context.g_motor_status[i].stall_cnt >= SAFETY_STALL_MAX_CNT) {
                g_sys_context.system_fault_code = FAULT_CODE_STALL;
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
        motor_pids[i].Out_Max  = dynamic_out_max;
        motor_pids[i].Out_Min  = -dynamic_out_max;
        motor_pids[i].Iout_Max = dynamic_iout_max;
    }

    float calc_target_v[4];
    float max_v = -1e9f;

    // 2. 算出 4 通道的理论 PID 调速结果 (直接引用全局上下文 g_sys_context 中无条件解算的绝对伸出高度)
    for (int i = 0; i < 4; i++) {
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
}

// ===================================================================
// 逐轴独立停稳检测与停机管理状态
// ===================================================================
static void APP_Control_SetLightOff(void);
static uint8_t s_axis_stable_cnt[4]   = {0, 0, 0, 0};                                     // 轴停稳计数器
static bool s_axis_is_settled[4]      = {false, false, false, false};                     // 轴停稳标志
static uint32_t s_last_check_halls[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}; // 轴上次检测绝对高度

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
    g_sys_context.active_motor_mask = 0x0F;
    g_sys_context.is_single_tuning  = false;
    g_sys_context.max_travel_diff   = 0;
    MID_Brake_Lock(); // 系统复位锁定抱闸自锁
}

/**
 * @brief 外部发起单轴微调
 */
void APP_Control_StartSingleTune(uint8_t m_idx)
{
    if (g_sys_context.system_step != SYS_STEP_READY || m_idx >= 4) return;

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
 * @brief 用户按键确认清除故障急停状态 (当 485 通信恢复/条件满足时)
 * @return true: 成功清除并进入归档自愈; false: 硬件通信故障仍存在，忽略清除
 */
bool APP_Control_ClearFault(void)
{
    if (g_sys_context.system_step != SYS_STEP_FAULT_STOP) {
        return false;
    }

    // 检查 4 轴 485 通信状态是否完全恢复正常
    bool comm_ok = true;
    for (int i = 0; i < 4; i++) {
        if (g_sys_context.g_motor_status[i].comm_error > 0) {
            comm_ok = false;
            break;
        }
    }

    if (comm_ok) {
        xQueueReset(g_motor_ctrl_queue);

        for (int i = 0; i < 4; i++) {
            g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
            g_sys_context.g_motor_status[i].target_speed = 0;
        }

        // 联动下发驱动器故障复位 (0x0007)，消除驱动器本体可能存在的过流/堵转报警红灯
        Motor_Ctrl_Msg_t reset_msg = {CMD_FAULT_RESET, 0x0F, 0};
        xQueueSend(g_motor_ctrl_queue, &reset_msg, pdMS_TO_TICKS(10));

        Motor_Ctrl_Msg_t stop_msg       = {CMD_STOP, 0x0F, 0};
        g_sys_context.system_fault_code = FAULT_CODE_NONE;
        APP_Control_PrepareStopSettling(0x0F, SYS_STEP_TOTAL_DONE);
        xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
        APP_Control_SetLightOff(); // 消除报警后关闭灯带
        Debug_Printf("[SYS] Fault Lockout Cleared by User Key! Sent CMD_FAULT_RESET & CMD_STOP to Motors...\r\n");
        return true;
    } else {
        Debug_Printf("[SYS] User Acknowledge Ignored: 485 Comm Fault Still Active!\r\n");
        return false;
    }
}

/**
 * @brief D 键智能化一键开关灯：如果有任一个开着，一次性关闭两个；如果都关着，一次性打开两个
 */
static void APP_Control_ToggleLightsByDKey(void)
{
    bool is1_on = MID_LED_ReadState(MID_LED_1);
    bool is2_on = MID_LED_ReadState(MID_LED_2);

    if (is1_on || is2_on) {
        // 只要有任意一个开着，一次性关闭两个
        MID_LED_Write(MID_LED_1, false);
        MID_LED_Write(MID_LED_2, false);
        Debug_Printf("[SYS] Remot D Key: Turning OFF both strip lights.\r\n");
    } else {
        // 两个都关着，一次性打开两个
        MID_LED_Write(MID_LED_1, true);
        MID_LED_Write(MID_LED_2, true);
        Debug_Printf("[SYS] Remot D Key: Turning ON both strip lights.\r\n");
    }
}

/**
 * @brief 正转/上升启动时一次性控制：打开第一个灯带，关闭第二个灯带
 */
static void APP_Control_SetLightForward(void)
{
    MID_LED_Write(MID_LED_1, true);  // 打开第一个灯带 (LED_1)
    MID_LED_Write(MID_LED_2, false); // 关闭第二个灯带 (LED_2)
}

/**
 * @brief 反转/下降启动时一次性控制：打开第二个灯带，关闭第一个灯带
 */
static void APP_Control_SetLightReverse(void)
{
    MID_LED_Write(MID_LED_1, false); // 关闭第一个灯带 (LED_1)
    MID_LED_Write(MID_LED_2, true);  // 打开第二个灯带 (LED_2)
}

/**
 * @brief 停机/到界/进入TOTAL_DONE时一次性控制：关掉两个灯带
 */
static void APP_Control_SetLightOff(void)
{
    MID_LED_Write(MID_LED_1, false); // 关闭第一个灯带
    MID_LED_Write(MID_LED_2, false); // 关闭第二个灯带
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
        g_sys_context.avg_delta_h += g_sys_context.delta_h[i];
        if (g_sys_context.delta_h[i] < min_dh) min_dh = g_sys_context.delta_h[i];
        if (g_sys_context.delta_h[i] > max_dh) max_dh = g_sys_context.delta_h[i];

        // B. 基于调平零点 (min_mount_halls) 的真正物理伸出行程 travel_rel[i] (用于 PID 闭环纠偏)
        g_sys_context.travel_rel[i] = (float)(calc_abs_hall - app_data.min_mount_halls[i]);
        g_sys_context.avg_travel += g_sys_context.travel_rel[i];
        if (g_sys_context.travel_rel[i] < min_tr) min_tr = g_sys_context.travel_rel[i];
        if (g_sys_context.travel_rel[i] > max_tr) max_tr = g_sys_context.travel_rel[i];
    }

    g_sys_context.avg_delta_h /= 4.0f;
    g_sys_context.max_dh_diff = max_dh - min_dh; // 本次单次运动位移增量极差

    g_sys_context.avg_travel /= 4.0f;
    g_sys_context.max_travel_diff = max_tr - min_tr; // 调平伸出行程极差 (伸出差)
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
    g_sys_context.active_motor_mask = 0x0F;
    g_sys_context.is_single_tuning  = false;
    g_sys_context.base_speed        = 0;
    g_sys_context.system_fault_code = 0;

    for (int i = 0; i < 4; i++) {
        APP_PID_Init(&motor_pids[i], PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD,
                     PID_DEFAULT_DEADZONE, PID_DEFAULT_OUT_MAX, PID_DEFAULT_OUT_MIN, PID_DEFAULT_IOUT_MAX);
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 非阻塞检查信号/事件
        bool has_event = MID_Signal_GetEvent(&sig_msg, 0);

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
                // 0. 待机状态实时 485 通信健康度巡检
                bool has_comm_loss = false;
                for (int i = 0; i < 4; i++) {
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
                    APP_Control_SetLightOff(); // 待机切入故障时先关闭普通指示灯，等待 1Hz 警示双闪
                    break;
                }

                // 检查遥控信号触发下行 / 上行
                if (has_event && sig_msg.event == MID_SIGNAL_EVT_TRIGGER) {
                    Motor_Ctrl_Msg_t speed_msg;
                    Motor_Ctrl_Msg_t cmd_msg;
                    bool action_valid = false;

                    int16_t run_rpm = (g_sys_context.calc_base_rpm > 0) ? g_sys_context.calc_base_rpm : 2400;

                    switch (sig_msg.signal_id) {
                        case MID_SIGNAL_REMOT_1: { // D 键 (REMOT_1): 一键控灯 (若有开则全关，无开则全亮)
                            APP_Control_ToggleLightsByDKey();
                            break;
                        }

                        case MID_SIGNAL_REMOT_3:
                        case MID_SIGNAL_BUTON_DW: { // B 键 (REMOT_3) & 外接信号下行 (PC7)
                            // 启动前安全检查：如果有任意轴已到达或低于底部零点 (travel_rel <= 0)，禁止启动下行！
                            bool limit_blocked = false;
                            for (int i = 0; i < 4; i++) {
                                float tr = (float)(g_sys_context.g_motor_status[i].current_abs_hall - app_data.min_mount_halls[i]);
                                if (tr <= 0.0f) {
                                    limit_blocked = true;
                                    Debug_Printf("[SYS] Down Start Blocked! Motor %d reached bottom limit (Travel=%.1f <= 0)\r\n", i, tr);
                                    break;
                                }
                            }
                            if (limit_blocked) break;

                            g_sys_context.is_single_tuning  = false;
                            g_sys_context.active_motor_mask = 0x0F;
                            g_sys_context.base_speed        = run_rpm;

                            speed_msg.cmd_type   = CMD_SET_SPEED;
                            speed_msg.motor_mask = 0x0F;
                            speed_msg.speed_rpm  = 300; // 起点转速 300 RPM

                            cmd_msg.cmd_type   = CMD_REVERSE;
                            cmd_msg.motor_mask = 0x0F;
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

                            APP_Control_SetLightReverse(); // 一次性控制：打开第二个灯带，关闭第一个灯带
                            action_valid              = true;
                            g_sys_context.ramp_cnt    = 0; // 重置 1000ms 缓启动计数
                            g_sys_context.system_step = SYS_STEP_TOTAL_RUNNING;
                            break;
                        }

                        case MID_SIGNAL_REMOT_4:
                        case MID_SIGNAL_BUTON_UP: { // 遥控上行 & 外接信号上行 (PC8)
                            // 启动前安全检查：如果有任意轴已到达或超过最大行程上限 (travel_rel >= max_travel_hall)，禁止启动上行！
                            bool limit_blocked = false;
                            for (int i = 0; i < 4; i++) {
                                float tr = (float)(g_sys_context.g_motor_status[i].current_abs_hall - app_data.min_mount_halls[i]);
                                if (tr >= (float)g_sys_context.max_travel_hall) {
                                    limit_blocked = true;
                                    Debug_Printf("[SYS] Up Start Blocked! Motor %d reached max travel limit (Travel=%.1f >= Limit=%d)\r\n",
                                                 i, tr, g_sys_context.max_travel_hall);
                                    break;
                                }
                            }
                            if (limit_blocked) break;

                            g_sys_context.is_single_tuning  = false;
                            g_sys_context.active_motor_mask = 0x0F;
                            g_sys_context.base_speed        = run_rpm;

                            speed_msg.cmd_type   = CMD_SET_SPEED;
                            speed_msg.motor_mask = 0x0F;
                            speed_msg.speed_rpm  = 300; // 起点转速 300 RPM

                            cmd_msg.cmd_type   = CMD_FORWARD;
                            cmd_msg.motor_mask = 0x0F;
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

                            APP_Control_SetLightForward(); // 一次性控制：打开第一个灯带，关闭第二个灯带
                            action_valid              = true;
                            g_sys_context.ramp_cnt    = 0; // 重置 1000ms 缓启动计数
                            g_sys_context.system_step = SYS_STEP_TOTAL_RUNNING;
                            break;
                        }

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

            // === SYS_STEP_SINGLE_TUNE 阶段：单轴微调运行中 ===
            case SYS_STEP_SINGLE_TUNE: {
                uint8_t m_idx       = g_sys_context.single_tune_motor_idx;
                uint32_t now_hall   = g_sys_context.g_motor_status[m_idx].hall_value;
                uint32_t start_hall = g_sys_context.single_tune_start_hall;

                // 0. 检查用户按键打断（如按 C 键停止）
                if (has_event && sig_msg.event == MID_SIGNAL_EVT_TRIGGER) {
                    if (sig_msg.signal_id == MID_SIGNAL_REMOT_2) {
                        APP_Control_CancelSingleTune();
                        Debug_Printf("[SYS] Single Tune Interrupted by User Remot C Key!\r\n");
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

                    APP_Control_PrepareStopSettling((uint8_t)(1 << m_idx), SYS_STEP_FAULT_STOP);
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

            // === 核心运行调速阶段（定频 20ms 数据驱动 PID 泵 + 安防防线） ===
            case SYS_STEP_TOTAL_RUNNING: {
                // 0. 4 轴采样屏障锁存等待 (Barrier Alignment)：若某轴霍尔恰好落后了微微秒 (正在接收 ACK)，让出 1ms 等其合流
                static uint32_t last_hall_seq[4] = {0};
                bool need_wait                   = false;
                for (int i = 0; i < 4; i++) {
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

                // 校验运行过程中的行程上下限到界判定 (到达边界自动软停机)
                bool is_running_forward = (g_sys_context.g_motor_status[0].target_cmd == CMD_FORWARD);
                bool is_running_reverse = (g_sys_context.g_motor_status[0].target_cmd == CMD_REVERSE);
                bool reach_limit        = false;

                if (is_running_forward) {
                    for (int i = 0; i < 4; i++) {
                        if (g_sys_context.travel_rel[i] >= (float)g_sys_context.max_travel_hall) {
                            reach_limit = true;
                            Debug_Printf("[SYS] Auto Stop: Motor %d reached max travel limit (%.1f >= %d)\r\n",
                                         i, g_sys_context.travel_rel[i], g_sys_context.max_travel_hall);
                            break;
                        }
                    }
                } else if (is_running_reverse) {
                    for (int i = 0; i < 4; i++) {
                        if (g_sys_context.travel_rel[i] <= 0.0f) {
                            reach_limit = true;
                            Debug_Printf("[SYS] Auto Stop: Motor %d reached zero bottom limit (%.1f <= 0)\r\n",
                                         i, g_sys_context.travel_rel[i]);
                            break;
                        }
                    }
                }

                if (has_event && sig_msg.event == MID_SIGNAL_EVT_TRIGGER) {
                    if (sig_msg.signal_id == MID_SIGNAL_REMOT_1) {
                        // D 键 (REMOT_1)：一键控灯 (若有开则全关，无开则全亮)，不断停运行
                        APP_Control_ToggleLightsByDKey();
                    } else if (sig_msg.signal_id == MID_SIGNAL_REMOT_2) {
                        // C 键 (REMOT_2)：专用停止按键，触发 CMD_STOP 停机！
                        xQueueReset(g_motor_ctrl_queue);

                        for (int i = 0; i < 4; i++) {
                            g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                            g_sys_context.g_motor_status[i].target_speed = 0;
                            last_sent_speed[i]                           = 0;
                        }

                        Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, 0x0F, 0};
                        APP_Control_PrepareStopSettling(0x0F, SYS_STEP_TOTAL_DONE);
                        xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                        APP_Control_SetLightOff(); // 停机一次性关两个灯
                        Debug_Printf("[SYS] Remot C Key (Stop Signal) Pressed, sending CMD_STOP...\r\n");
                    }
                } else if (reach_limit) {
                    // 触及行程上限或零点到界停机
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }

                    Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, 0x0F, 0};
                    APP_Control_PrepareStopSettling(0x0F, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    APP_Control_SetLightOff(); // 到界停机一次性关两个灯
                } else if (APP_Control_CheckSafety()) {
                    xQueueReset(g_motor_ctrl_queue);
                    if (g_sys_context.system_fault_code == FAULT_CODE_STALL) {
                        // 启动 4 轴整体以正常速度的一半 (calc_base_rpm / 2) 反弹用户可调高度 app_data.rebound_travel_mm (默认1000mm=1米)
                        uint8_t current_cmd                 = g_sys_context.g_motor_status[0].target_cmd;
                        g_sys_context.rebound_cmd           = (current_cmd == CMD_FORWARD) ? CMD_REVERSE : CMD_FORWARD;
                        g_sys_context.rebound_target_counts = (uint32_t)roundf((float)app_data.rebound_travel_mm * g_sys_context.counts_per_mm);
                        g_sys_context.active_motor_mask     = 0x0F;

                        // 反弹速度设为正常计算速度的一半 (若过小则保底 100 RPM)
                        uint16_t rebound_speed_rpm = g_sys_context.calc_base_rpm / 2;
                        if (rebound_speed_rpm < 100) rebound_speed_rpm = 100;

                        for (int i = 0; i < 4; i++) {
                            g_sys_context.rebound_start_hall[i]              = g_sys_context.g_motor_status[i].hall_value;
                            g_sys_context.g_motor_status[i].start_drive_hall = g_sys_context.g_motor_status[i].hall_value;
                            g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                            g_sys_context.g_motor_status[i].target_cmd       = g_sys_context.rebound_cmd;
                            g_sys_context.g_motor_status[i].target_speed     = rebound_speed_rpm;
                            g_sys_context.g_motor_status[i].stall_cnt        = 0;
                        }

                        Motor_Ctrl_Msg_t speed_msg = {CMD_SET_SPEED, 0x0F, rebound_speed_rpm};
                        Motor_Ctrl_Msg_t cmd_msg   = {(Motor_Cmd_Type_t)g_sys_context.rebound_cmd, 0x0F, 0};

                        MID_Brake_Release(); // 453 机械抱闸：确保抱闸处于释放状态

                        xQueueSend(g_motor_ctrl_queue, &speed_msg, pdMS_TO_TICKS(10));
                        xQueueSend(g_motor_ctrl_queue, &cmd_msg, pdMS_TO_TICKS(10));

                        g_sys_context.system_step = SYS_STEP_TOTAL_REBOUND;
                        Debug_Printf("[SYS] OverCurrent Stall! Starting Half-Speed Rebound: Speed=%dRPM, Distance=%dmm (TargetCounts=%d)...\r\n",
                                     rebound_speed_rpm, app_data.rebound_travel_mm, g_sys_context.rebound_target_counts);
                    } else {
                        // 通信中断/同步差超限：先下发全轴 STOP 切入 SYS_STEP_TOTAL_DONE 进行刹车停稳与绝对位置归档！
                        bool has_comm_err = false;
                        for (int i = 0; i < 4; i++) {
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
                        Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, 0x0F, 0};
                        APP_Control_PrepareStopSettling(0x0F, SYS_STEP_TOTAL_DONE);
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

                    // 3. 极速 PID 位置同步计算 (动态限幅 + 防饱和速度平移)
                    APP_Control_RunPID(run_base_speed);

                    // 4. 【按需门限下发】转速变动幅度 >= 2 RPM 时才向队列推送写速度命令 (防抖且绝对不压制 PID)
                    for (int i = 0; i < 4; i++) {
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
                // 1. 无条件调用通用解算函数更新 4 轴绝对高度、伸出行程及极差
                APP_Control_UpdateStateAndStatistics();

                // 2. 安防防线 1：485 通信中断检查
                bool has_comm_err = false;
                for (int i = 0; i < 4; i++) {
                    if (g_sys_context.g_motor_status[i].comm_error >= SAFETY_COMM_ERR_MAX_CNT) {
                        has_comm_err = true;
                        Debug_Printf("[ERR] Rebound Comm Loss! Motor %d (CommErr=%d >= Limit=%d)\r\n",
                                     i + 1, g_sys_context.g_motor_status[i].comm_error, SAFETY_COMM_ERR_MAX_CNT);
                        break;
                    }
                }
                if (has_comm_err) {
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg       = {CMD_STOP, 0x0F, 0};
                    g_sys_context.system_fault_code = FAULT_CODE_COMM;
                    APP_Control_PrepareStopSettling(0x0F, SYS_STEP_FAULT_STOP);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[SYS] Rebound Aborted: Comm Loss! Entering FAULT_STOP.\r\n");
                    break;
                }

                // 3. 安防防线 2：反弹过程中四轴同步差超限（防止个别柱子卡死不退导致台面严重倾斜拉偏）
                if (g_sys_context.max_travel_diff > (float)g_sys_context.max_sync_diff_hall) {
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg       = {CMD_STOP, 0x0F, 0};
                    g_sys_context.system_fault_code = FAULT_CODE_REBOUND_SYNC;
                    APP_Control_PrepareStopSettling(0x0F, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[ERR] Rebound Sync Diff Exceeded (Diff=%.1f > Limit=%d)! Stopping Rebound & Entering SYS_STEP_TOTAL_DONE to Auto-Align...\r\n",
                                 g_sys_context.max_travel_diff, g_sys_context.max_sync_diff_hall);
                    break;
                }

                // 4. 安防防线 3：反弹过程中二次过流堵转检查 (反向运动受阻)
                bool has_rebound_stall = false;
                for (int i = 0; i < 4; i++) {
                    if (g_sys_context.g_motor_status[i].current_deciA > app_data.stall_current_threshold) {
                        g_sys_context.g_motor_status[i].stall_cnt++;
                        if (g_sys_context.g_motor_status[i].stall_cnt >= SAFETY_STALL_MAX_CNT) {
                            has_rebound_stall = true;
                            Debug_Printf("[ERR] Rebound Second Stall! Motor %d (Curr=%.2fA > Limit=%.2fA)\r\n",
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
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }
                    Motor_Ctrl_Msg_t stop_msg       = {CMD_STOP, 0x0F, 0};
                    g_sys_context.system_fault_code = FAULT_CODE_REBOUND_STALL;
                    APP_Control_PrepareStopSettling(0x0F, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[SYS] Rebound Second Stall Detected! Stopping Rebound & Entering SYS_STEP_TOTAL_DONE to Auto-Align/Lockout...\r\n");
                    break;
                }

                // 5. 解算 4 轴反弹累计增量 (计算平均反弹步数)
                float sum_reb_counts = 0.0f;
                for (int i = 0; i < 4; i++) {
                    int32_t reb_h = g_sys_context.g_motor_status[i].current_abs_hall - g_sys_context.g_motor_status[i].base_abs_hall;
                    sum_reb_counts += (reb_h >= 0) ? (float)reb_h : (float)(-reb_h);
                }
                float avg_reb_counts = sum_reb_counts / 4.0f;

                // 6. 堵转反弹阶段实时 PID 纠偏调速 (保持顶部台面绝对平行，采用正常速度的一半)
                uint16_t rebound_speed_rpm = g_sys_context.calc_base_rpm / 2;
                if (rebound_speed_rpm < 100) rebound_speed_rpm = 100;
                APP_Control_RunPID(rebound_speed_rpm);

                // 【按需门限下发】向 4 轴下发 PID 平行纠偏调整速度
                for (int i = 0; i < 4; i++) {
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

                if (has_event && sig_msg.event == MID_SIGNAL_EVT_TRIGGER) {
                    if (sig_msg.signal_id == MID_SIGNAL_REMOT_1) {
                        // D 键 (REMOT_1)：一键控灯，不断停反弹
                        APP_Control_ToggleLightsByDKey();
                    } else if (sig_msg.signal_id == MID_SIGNAL_REMOT_2) {
                        // C 键 (REMOT_2)：专用停止按键，手动打断反弹流程！
                        rebound_done = true;
                        Debug_Printf("[SYS] Rebound Interrupted by User Remot C Key!\r\n");
                    }
                }

                if (!rebound_done) {
                    if (g_sys_context.rebound_cmd == CMD_REVERSE) {
                        for (int i = 0; i < 4; i++) {
                            if (g_sys_context.travel_rel[i] <= 0.0f) {
                                rebound_done = true;
                                break;
                            }
                        }
                    } else if (g_sys_context.rebound_cmd == CMD_FORWARD) {
                        for (int i = 0; i < 4; i++) {
                            if (g_sys_context.travel_rel[i] >= (float)g_sys_context.max_travel_hall) {
                                rebound_done = true;
                                break;
                            }
                        }
                    }
                }

                // 8. 反弹完成：直接下发全轴 STOP，切入 SYS_STEP_TOTAL_DONE 统一接管停稳判定与 Flash 固化！
                if (rebound_done) {
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }

                    Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, 0x0F, 0};
                    APP_Control_PrepareStopSettling(0x0F, SYS_STEP_TOTAL_DONE);
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[SYS] Rebound Distance Reached (AvgCounts=%.0f), Sending CMD_STOP & Entering SYS_STEP_TOTAL_DONE...\r\n", avg_reb_counts);
                }

                APP_Control_DebugPrint();
                break;
            }

            // === 停机归档阶段（逐轴独立停稳检测与停机 + 运动轴全停后单次 Flash 归档） ===
            case SYS_STEP_TOTAL_DONE:
            case SYS_STEP_TUNE_DONE: {
                APP_Control_SetLightOff(); // 停机刹车阶段一次性关闭 2 路灯带
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

                    // 立即同步固化存 Flash（仅执行 1 次落盘）
                    APP_Data_Storage();
                    MID_Brake_Lock(); // 453 机械抱闸安全时序：运动轴彻底停稳后断电抱紧自锁
                    Debug_Printf("[SYS] All Active Motors Settled & Flash Archived Successfully!\r\n");

                    // 以最新的 min_mount_halls 重新计算 4 轴绝对伸出行程 travel_rel 及极差 max_travel_diff
                    APP_Control_UpdateStateAndStatistics();

                    // 重置 4 轴 PID 控制器历史状态
                    for (int i = 0; i < 4; i++) {
                        APP_PID_Init(&motor_pids[i], PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD,
                                     PID_DEFAULT_DEADZONE, PID_DEFAULT_OUT_MAX, PID_DEFAULT_OUT_MIN, PID_DEFAULT_IOUT_MAX);
                    }

                    g_sys_context.base_speed = 0;
                    APP_Control_DebugPrint();

                    // 4. 停稳归档完成后的流转决策 (故障 / 自愈对齐 / 就绪)
                    bool has_comm_fault = false;
                    for (int i = 0; i < 4; i++) {
                        if (g_sys_context.g_motor_status[i].comm_error > 0) {
                            has_comm_fault = true;
                            break;
                        }
                    }

                    if (has_comm_fault || g_sys_context.system_fault_code == FAULT_CODE_COMM) {
                        g_sys_context.system_fault_code = FAULT_CODE_COMM;
                        g_sys_context.system_step       = SYS_STEP_FAULT_STOP;
                        MID_Brake_Lock(); // 致命通信故障抱死自锁
                        Debug_Printf("[SYS] Stop Check: 485 Comm Fault Active! Entering SYS_STEP_FAULT_STOP Lockout State.\r\n");
                    } else if (g_sys_context.max_travel_diff > (float)g_sys_context.max_sync_diff_hall ||
                               g_sys_context.system_fault_code == FAULT_CODE_REBOUND_SYNC ||
                               g_sys_context.system_fault_code == FAULT_CODE_REBOUND_STALL) {
                        for (int i = 0; i < 4; i++) {
                            g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                            g_sys_context.g_motor_status[i].start_drive_hall = g_sys_context.g_motor_status[i].hall_value;
                        }
                        g_sys_context.system_step = SYS_STEP_AUTO_ALIGN;
                        MID_Brake_Release();           // 453 抱闸控制：启动自愈重平前通电松开抱闸
                        vTaskDelay(pdMS_TO_TICKS(80)); // 硬件脱开延时 80ms
                        Debug_Printf("[SYS] Stop Check: Sync Diff Exceeded or Rebound Fault (FaultCode=%d, Diff=%.1f > Limit=%d)! Triggering AUTO_ALIGN Self-Healing...\r\n",
                                     g_sys_context.system_fault_code, g_sys_context.max_travel_diff, g_sys_context.max_sync_diff_hall);
                    } else {
                        g_sys_context.system_fault_code = FAULT_CODE_NONE;
                        g_sys_context.system_step       = SYS_STEP_READY;
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
                // 1. 实时解算 4 轴绝对高度与伸出行程 travel_rel
                APP_Control_UpdateStateAndStatistics();

                // 2. 检查极差收敛对齐终止条件 (收敛至 max_sync_diff_hall * AUTO_ALIGN_TARGET_DIFF_RATIO 以内)
                float target_converge_diff = (float)g_sys_context.max_sync_diff_hall * AUTO_ALIGN_TARGET_DIFF_RATIO;
                if (g_sys_context.max_travel_diff <= target_converge_diff) {
                    xQueueReset(g_motor_ctrl_queue);
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }

                    Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, 0x0F, 0};
                    APP_Control_PrepareStopSettling(0x0F, SYS_STEP_TUNE_DONE);
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

            // === SYS_STEP_FAULT_STOP 阶段：致命故障急停锁死状态 (报警显示、通信恢复安全下发 STOP 与用户确认解锁归档) ===
            case SYS_STEP_FAULT_STOP: {
                // 1. 检查 4 轴 485 通信状态是否完全恢复正常
                bool comm_ok = true;
                for (int i = 0; i < 4; i++) {
                    if (g_sys_context.g_motor_status[i].comm_error > 0) {
                        comm_ok = false;
                        break;
                    }
                }

                static bool has_sent_stop_on_comm_restore = false;

                // 若通信中断未恢复，重置标志
                if (!comm_ok) {
                    has_sent_stop_on_comm_restore = false;
                }

                // 2. 场景 A：重新插上数据线 / 重新上电复位，通信恢复时【仅发送 STOP 停机，保持报警锁死状态】
                if (comm_ok && !has_sent_stop_on_comm_restore) {
                    has_sent_stop_on_comm_restore = true;
                    xQueueReset(g_motor_ctrl_queue);

                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                        last_sent_speed[i]                           = 0;
                    }

                    // 通信恢复时先下发故障复位清除驱动器潜在红灯，随后确保停机
                    Motor_Ctrl_Msg_t reset_msg = {CMD_FAULT_RESET, 0x0F, 0};
                    xQueueSend(g_motor_ctrl_queue, &reset_msg, pdMS_TO_TICKS(10));

                    Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, 0x0F, 0};
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, pdMS_TO_TICKS(10));
                    Debug_Printf("[SYS] 485 Comm Restored: Sent CMD_FAULT_RESET & CMD_STOP to Motors, Keeping FAULT_STOP Alarm Active Until User Acknowledge...\r\n");
                }

                // 3. 场景 B：遥控/外接信号按键触发取消报警
                if (has_event && (sig_msg.event == MID_SIGNAL_EVT_TRIGGER || sig_msg.event == MID_SIGNAL_EVT_LONG)) {
                    if (APP_Control_ClearFault()) {
                        has_sent_stop_on_comm_restore = false;
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
