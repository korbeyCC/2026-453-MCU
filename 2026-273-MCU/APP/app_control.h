#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "app_main.h"

// ===================================================================
// 系统级安防与故障判定宏配置
// ===================================================================
#define SAFETY_COMM_ERR_MAX_CNT 10 // 通信连续中断判定次数 (连续 10 帧/40ms 无回应触发保护)
#define SAFETY_STALL_MAX_CNT    10 // 堵转过流判定持续次数 (20 帧 x 20ms = 400ms 持续过流触发堵转)

// ===================================================================
// 系统运行速度与起跑斜坡宏配置
// ===================================================================
#define RAMP_UP_TOTAL_STEPS          50  // 1000ms 缓启动递增总步数 (50 帧 x 20ms = 1000ms)
#define RAMP_UP_START_RPM            300 // 缓启动起跑初始转速 (RPM)

#define MOTOR_MIN_RUN_RPM            300  // 电机运行最低允许转速 (RPM)
#define MOTOR_MAX_RUN_RPM            3000 // 电机运行最高上限转速 (RPM)

#define REBOUND_TUNE_RPM             300   // 堵转反弹及微调运行转速 (RPM)
#define REBOUND_DISTANCE_MM          30.0f // 堵转后反弹后退距离 (mm)

#define STOP_STABLE_CHECK_CNT        10 // 停机归档阶段连续静止确认次数 (10 帧 x 20ms = 200ms)

#define AUTO_ALIGN_SPEED_RPM         300  // 四轴自主台面平行恢复基准转速 (RPM)
#define AUTO_ALIGN_TARGET_DIFF_RATIO 0.3f // 对齐极差目标收敛比例 (收敛至 max_sync_diff_hall * 0.3 以内完成)
#define AUTO_ALIGN_TIMEOUT_SEC       6    // 调平无进展看门狗超时时间 (秒)
#define AUTO_ALIGN_TIMEOUT_TICKS     (AUTO_ALIGN_TIMEOUT_SEC * 1000 / 20) // 调平无进展看门狗总帧数 (6s * 1000 / 20ms = 300 帧)

// 系统故障代码定义 (用于全系统状态监控与数码管 ErrX 报警显示)
#define FAULT_CODE_NONE         0 // 正常无故障
#define FAULT_CODE_STALL        1 // 过流堵转 (Err1: 上升堵转、微调堵转、调平堵转、反弹中再过流)
#define FAULT_CODE_COMM         2 // 485 通信中断故障 (Err2)
#define FAULT_CODE_SYNC         3 // 运行同步差超限/调平超时发散故障 (Err3)
#define FAULT_CODE_PINCH        4 // 下降防夹反弹故障 (Err4: 仅下降受阻触发防夹反弹脱困)
#define FAULT_CODE_DRIVER_ALARM 6 // 驱动器本体报警 (Err6, 内部状态字置位故障中)

// ===================================================================
// 动态 PID 纠偏与基础初始化宏配置
// ===================================================================
#define PID_DEFAULT_KP          0.60f   // 默认比例增益 Kp
#define PID_DEFAULT_KI          0.001f  // 默认积分增益 Ki
#define PID_DEFAULT_KD          0.0f    // 默认微分增益 Kd
#define PID_DEFAULT_DEADZONE    0.0f    // 默认控制死区 (counts)
#define PID_DEFAULT_OUT_MAX     100.0f  // 默认输出限幅 (RPM)
#define PID_DEFAULT_OUT_MIN     -100.0f // 默认输出下限 (RPM)
#define PID_DEFAULT_IOUT_MAX    30.0f   // 默认积分限幅 (RPM)

#define PID_DIFF_LOW_THRESHOLD  50.0f  // PID 动态限幅低偏差门限 (counts, 约 0.44mm)
#define PID_DIFF_HIGH_THRESHOLD 300.0f // PID 动态限幅高偏差门限 (counts, 约 2.66mm)

#define PID_OUT_MAX_LOW         100.0f // 小偏差时 PID 最高调节转速 (RPM)
#define PID_OUT_MAX_HIGH        600.0f // 大偏差时 PID 极速拉平转速 (RPM)
#define PID_IOUT_MAX_LOW        30.0f  // 小偏差时 PID 积分限幅 (RPM)
#define PID_IOUT_MAX_HIGH       150.0f // 大偏差时 PID 积分限幅 (RPM)

/* 系统整体运行流程状态机 */
typedef enum {
    SYS_STEP_Boot = 0, // 系统刚上电引导启动（等待硬件初始化）
    SYS_STEP_READY,    // 系统配置完成，就绪待命（等待输入）

    SYS_STEP_SINGLE_TUNE, // 单路立柱微调控制中
    SYS_STEP_TOTAL_TUNE,  // 四柱一键同步微调控制中 (增量平行 PID + 逐轴定长截断)
    SYS_STEP_TUNE_DONE,   // 微调/停机结束 (Flash 归档与对齐校验)
    SYS_STEP_AUTO_ALIGN,  // 四轴自主台面平行恢复 (自愈重平控制中)

    SYS_STEP_TOTAL_RUNNING, // 整体运行调速阶段（定频 PID 纠偏泵）
    SYS_STEP_TOTAL_REBOUND, // 仅下降防夹触发的反方向反弹阶段 (退避 30mm)
    SYS_STEP_TOTAL_DONE,    // 整体结束停机中 (逐轴静止判定与 Flash 归档)
    SYS_STEP_FAULT_STOP     // 致命故障急停状态 (抱闸咬死自锁)
} Motor_ctl_Step_t;

/* 电机单通道运行与状态监控 */
typedef struct {
    int32_t base_abs_hall;       // 本次运行起步前的绝对高度基准 (有符号)
    uint32_t start_drive_hall;   // 本次起步时驱动器的原始无符号读数起点 (无符号 32 位)
    int32_t current_abs_hall;    // 实时解算的绝对高度 (有符号，用于 PID 控制)
    uint32_t hall_value;         // 驱动器最新原始无符号霍尔读数 (内存数据缓存)
    uint16_t current_deciA;      // 驱动器当前输出电流 (单位: 0.01A)
    uint16_t driver_status_word; // 驱动器状态字 1 (0x2100: 1正转 2反转 3停机 4故障 5OFF)
    uint16_t driver_fault_code;  // 驱动器当前故障代码 (0x2102: 0无故障, 1~36对应驱动器故障表)
    uint8_t stall_cnt;           // 过流堵转判定计数器
    uint8_t comm_error;          // 通信超时错误标记
    uint8_t retry_cnt;           // 重试计数

    uint8_t target_cmd;      // 目标动作指令
    int16_t target_speed;    // 目标转速
    uint8_t current_cmd;     // 驱动器当前真实的运行指令
    int16_t current_speed;   // 驱动器当前真实的转速配置
    uint8_t last_motion_cmd; // 本次起步/运动的有效物理方向 (CMD_FORWARD 或 CMD_REVERSE)
} Motor_Status_t;

/* 全局控制上下文 (解耦数据驱动架构) */
typedef struct {
    Motor_ctl_Step_t system_step;     // 系统级运行状态机
    Motor_Status_t g_motor_status[4]; // 4路立柱电机状态快照
    volatile bool is_hardware_ready;  // 4路 Modbus 硬件初始化完成标志
    int16_t base_speed;               // 系统全局基准转速 (RPM)
    uint16_t ramp_cnt;                // 300ms 缓启动递增计数器 (0~60)

    // 1. 当帧位移增量与其全局统计量共享字段
    float delta_h[4];  // 4 轴当帧最新的位移增量 ΔH_i
    float avg_delta_h; // 4 轴当帧平均位移增量 ΔH_avg
    float max_dh_diff; // 4 轴当帧最大轴间增量偏差 (max - min)

    // 2. 基于调平零点 (min_mount_halls) 的绝对伸出高度及其统计量 (专用于 PID 绝对纠偏)
    float travel_rel[4];   // 4 轴当帧绝对伸出行程 (counts)
    float avg_travel;      // 4 轴当帧平均绝对伸出行程 (counts)
    float max_travel_diff; // 4 轴当帧最大绝对高度差 (max - min)

    // 自动物理换算参数与安防状态
    float counts_per_mm;                  // 每 mm 霍尔计数值 (支持 112.5f 浮点精度)
    int32_t max_sync_diff_hall;           // 最大同步差霍尔计数值
    int32_t max_travel_hall;              // 最大行程霍尔计数值
    int16_t calc_base_rpm;                // 算出的基准 RPM
    uint8_t system_fault_code;            // 故障代码 (0:正常, 1:过流堵转, 2:通信中断, 3:同步差超限)
    volatile uint32_t hall_update_seq[4]; // 4 轴霍尔成功更新打卡序列号

    // 单轴微调与过流反弹控制状态字段
    uint8_t active_motor_mask;          // 当前运动参与的电机掩码 (单轴微调为 1<<m_idx, 四轴联动为 0x0F)
    volatile bool is_single_tuning;     // 是否正在执行单轴微调标志 (严格防护 Flash min_mount_halls 污染)
    uint8_t single_tune_dir;            // 微调方向 (0: 正转/上升, 1: 反转/下降)
    uint8_t single_tune_motor_idx;      // 当前微调的目标电机 (0~3)
    uint32_t single_tune_start_hall;    // 微调开始时的驱动器原始霍尔起点
    int32_t single_tune_orig_abs_hall;  // 微调开始时的起点绝对霍尔高度
    uint32_t single_tune_target_counts; // 单轴微调目标霍尔步计数
    // 四柱同步一键微调状态字段
    volatile bool is_total_tuning;        // 是否正在执行四柱同步微调标志
    uint32_t total_tune_target_counts;    // 四柱微调目标霍尔步计数

    uint8_t rebound_cmd;            // 反弹运动指令 (CMD_FORWARD 或 CMD_REVERSE)
    uint32_t rebound_start_hall[4]; // 反弹开始时 4 轴原始霍尔读数
    uint32_t rebound_target_counts; // 反弹目标霍尔增量计数 (30mm * counts_per_mm)
} Sys_Ctrl_Context_t;

// 全局外部变量声明
extern Sys_Ctrl_Context_t g_sys_context;
extern bool g_manual_light_on; // D 键手动控制灯带一键开关标志 (true: 强行全亮, false: 跟随方向指示/全灭)

void APP_Control_UpdateParamsFromAppData(void);
void APP_Control_ResetSystemContext(void);
void APP_Control_UpdateStateAndStatistics(void);
void APP_Control_StartSingleTune(uint8_t m_idx);
void APP_Control_CancelSingleTune(void);
void APP_Control_StartTotalTune(void);
bool APP_Control_ClearFault(void);
void APP_Control_EmergencyStop(void);
void APP_ControlTask(void *pvParameters);

#endif // __APP_CONTROL_H
