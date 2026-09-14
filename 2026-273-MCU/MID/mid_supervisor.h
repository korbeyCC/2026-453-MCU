#ifndef __MID_SUPERVISOR_H
#define __MID_SUPERVISOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==============================================================================
// 跨平台与跨项目复用开关 (0: 纯 C 裸机 / 8051 / 裸机 OSAL, 1: FreeRTOS 环境)
// ==============================================================================
#define SYS_SUPERVISOR_USE_FREERTOS 1

#if SYS_SUPERVISOR_USE_FREERTOS
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#endif

// ==============================================================================
// 1. 系统顶级运行模式 (System Operation Mode) —— 单一可信源 (SSOT)
// ==============================================================================
typedef enum {
    SYS_MODE_STANDBY = 0,     // 待机就绪态 (主界面, 4轴读数监测, 接受运动与微调, 允许进菜单)
    SYS_MODE_MOTION,          // 闭环运行态 (正在升降/微调中, 锁定菜单入口)
    SYS_MODE_AUTO_ALIGN,      // 自动调平态 (专用算法闭环, 屏蔽常规升降输入)
    SYS_MODE_MENU_CONFIG,     // 菜单调参态 (按键独占K1~K6, 电机强制闭锁停机, 防误动)
    SYS_MODE_DEBUG_CALIB,     // 深层调试态 (纯只读安装零点mm)
    SYS_MODE_FAULT_LOCKED     // 故障锁定态 (报警闪烁落盘, 仅响应消警与复位)
} System_Mode_t;

// ==============================================================================
// 2. 指示灯与蜂鸣器联动状态定义 (供 app_show 纯读使用)
// ==============================================================================
typedef enum {
    SYS_IND_FAULT = 0,        // 故障急停双闪 (1Hz/2Hz 警示闪烁 + 报警鸣叫)
    SYS_IND_RUNNING,          // 运行中指示 (工作指示灯连续闪烁 + 运动蜂鸣)
    SYS_IND_STEADY            // 待机就绪态 (指示灯常亮 + 静音)
} Sys_Indicator_State_t;

// ==============================================================================
// 3. 统一事件源与事件对象定义 (Sys_Event_t)
// ==============================================================================
typedef enum {
    SYS_EVT_SRC_KEY = 0,      // 面板物理按键 (如 mid_Key)
    SYS_EVT_SRC_SIGNAL,       // 无线遥控器 / 外部干接点 (如 mid_signal)
    SYS_EVT_SRC_MODBUS,       // 串口 / 485 总线主令
    SYS_EVT_SRC_FAULT         // 硬件保护中断 / 底层异常
} Sys_Event_Source_t;

typedef struct {
    uint8_t source;           // 事件来源: Sys_Event_Source_t
    uint8_t id;               // 实体 ID: 键值 ID 或 遥控通道号
    uint8_t event_type;       // 动作类型: PRESS, RELEASE, LONG, REPEAT
    uint8_t count;            // 连击次数 / 扩展参数
    uint32_t param;           // 32 位扩展载荷 (可选)
} Sys_Event_t;

// ==============================================================================
// 4. 优先级拦截管道过滤结果 (Priority Filter Result)
// ==============================================================================
typedef enum {
    EVENT_PASS_THROUGH = 0,   // 放行：本层不处理或不拦截，漏给下一优先级
    EVENT_CONSUMED     = 1    // 消费并拦截：本层已处理，就地销毁，管道终止！
} Event_Result_t;

// 管道节点处理函数原型
typedef Event_Result_t (*Sys_Pipeline_Filter_t)(const Sys_Event_t *p_evt);

// ==============================================================================
// 5. 运动控制单值覆盖邮箱 (Overwrite Mailbox) —— 绝无队列积压滞后
// ==============================================================================
typedef enum {
    SYS_MOTION_SRC_NONE = 0,
    SYS_MOTION_SRC_SIGNAL,    // 遥控器输入 (MID_SIGNAL_REMOT_X)
    SYS_MOTION_SRC_KEY        // 面板按键动作输入 (MID_KEY_ID_K1~K4 微调动作 或 打断按键)
} Sys_Motion_Source_t;

typedef struct {
    volatile uint8_t src;        // 来源: Sys_Motion_Source_t
    volatile uint8_t id;         // signal_id 或 key_id
    volatile uint8_t event_type; // 动作类型
    volatile bool    has_cmd;
} Sys_Motion_Mailbox_t;

// ==============================================================================
// 6. 核心门禁与仲裁 API
// ==============================================================================

/**
 * @brief  初始化模式仲裁器与事件管道
 */
void Sys_Supervisor_Init(void);

/**
 * @brief  事件进入优先级管道自顶向下分发 (栈直调入口，0 RAM 纳秒级响应)
 * @param  p_evt: 统一事件对象指针
 * @return EVENT_CONSUMED 表示被某一优先级拦截消费；EVENT_PASS_THROUGH 表示全管道穿透无人认领
 */
Event_Result_t Sys_Event_Dispatch(const Sys_Event_t *p_evt);

/**
 * @brief  切换系统全局顶级主模式
 */
void Sys_Mode_Set(System_Mode_t new_mode);

/**
 * @brief  获取当前系统全局顶级主模式
 */
System_Mode_t Sys_Mode_Get(void);

/**
 * @brief  权威门禁：当前是否允许启动电机运动
 * @return true 允许; false 严格禁止 (如正在调参菜单中、或处于故障锁定中)
 */
bool Sys_Mode_CanRunMotion(void);

/**
 * @brief  权威门禁：当前是否允许进入参数设置菜单
 * @return true 允许; false 严格禁止 (如电机正在高速运动或调平中)
 */
bool Sys_Mode_CanEnterMenu(void);

/**
 * @brief  查询当前菜单是否正处于调参活跃状态 (独占按键)
 */
bool Sys_Mode_IsMenuActive(void);

/**
 * @brief  查询当前是否处于严重故障锁定状态
 */
bool Sys_Mode_IsFaultLocked(void);

// ==============================================================================
// 7. 多载体交付适配接口 (Multi-Carrier Sinks)
// ==============================================================================

// --- 载体 A: 单值覆盖邮箱 (供控制任务读取运动指令，只留最新值) ---
void Sys_Mailbox_PostMotionCmd(uint8_t src, uint8_t id, uint8_t event_type);
bool Sys_Mailbox_GetMotionCmd(uint8_t *p_src, uint8_t *p_id, uint8_t *p_event_type);
void Sys_Mailbox_ClearMotionCmd(void);

// --- 载体 B: 任务通知 (用于菜单按键唤醒，0 额外 RAM 开销) ---
#if SYS_SUPERVISOR_USE_FREERTOS
void Sys_Supervisor_RegisterMenuTask(TaskHandle_t task_handle);
bool Sys_Supervisor_NotifyMenuKey(uint8_t key_id, uint8_t event_type, uint8_t count);
bool Sys_Supervisor_WaitMenuKey(uint8_t *p_key_id, uint8_t *p_event_type, uint8_t *p_count, uint32_t timeout_ms);
#endif

// ==============================================================================
// 8. 系统只读视图模型接口 (System View Model / Telemetry) —— 供 app_show 纯读使用
// ==============================================================================

/**
 * @brief  获取指定立柱电机的实时绝对伸出高度 (单位: mm, 已换算四舍五入)
 * @param  axis_idx: 电机通道号 0 ~ 3
 * @return 绝对伸出高度 mm (0 ~ 9999)
 */
int32_t Sys_View_GetAxisTravelMm(uint8_t axis_idx);

/**
 * @brief  获取指定立柱电机的基准安装零点高度 (单位: mm, 已换算四舍五入)
 * @param  axis_idx: 电机通道号 0 ~ 3
 * @return 安装零点高度 mm
 */
int32_t Sys_View_GetAxisMountMm(uint8_t axis_idx);

/**
 * @brief  获取当前系统有效故障代码 (0 表示正常无故障, 1:堵转, 2:通讯, 3:同步, 4:防夹, 6:驱动器)
 */
uint8_t Sys_View_GetFaultCode(void);

/**
 * @brief  查询系统当前是否处于防夹反弹运行中 (用于 500ms 高度 <-> 500ms Err4 交替显示)
 */
bool Sys_View_IsRebounding(void);

/**
 * @brief  获取指示灯与蜂鸣器的中立模式 (故障闪烁/运行中/常亮静音)
 */
Sys_Indicator_State_t Sys_View_GetIndicatorState(void);

/**
 * @brief  设置单轴微调方向 (0: 正转/上升 -UP-, 1: 反转/下降 -dn-)
 */
void Sys_View_SetTuneDir(uint8_t dir);

/**
 * @brief  获取单轴微调方向
 */
uint8_t Sys_View_GetTuneDir(void);

// ==============================================================================
// 9. 中立通知与请求接口 (供 app_Menu 纯发意图使用)
// ==============================================================================

/**
 * @brief  通知底层：菜单参数已更新，需重新换算 counts_per_mm 与目标转速
 */
void Sys_Notify_ParamsUpdated(void);

/**
 * @brief  通知底层：菜单已触发恢复出厂设置，需全面重置系统状态机与4轴位置
 */
void Sys_Notify_FactoryReset(void);

/**
 * @brief  请求系统清除当前故障急停锁死状态
 * @return true 成功清除并下发停机归档; false 驱动器故障未恢复，拒绝清除
 */
bool Sys_Request_ClearFault(void);

#ifdef __cplusplus
}
#endif

#endif // __MID_SUPERVISOR_H
