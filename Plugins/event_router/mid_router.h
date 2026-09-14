#ifndef __MID_ROUTER_H
#define __MID_ROUTER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==============================================================================
// 跨平台与跨项目复用开关 (0: 纯 C 裸机 / 8051 / 裸机 OSAL, 1: FreeRTOS 环境)
// ==============================================================================
#define SYS_ROUTER_USE_FREERTOS 1

#if SYS_ROUTER_USE_FREERTOS
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#endif

// ==============================================================================
// 1. 统一事件源与事件对象定义 (Sys_Event_t)
// ==============================================================================
typedef enum {
    SYS_EVT_SRC_KEY = 0,      // 面板物理按键 (如 mid_Key)
    SYS_EVT_SRC_SIGNAL,       // 无线遥控器 / 外部干接点 (如 mid_signal)
    SYS_EVT_SRC_MODBUS,       // 串口 / 485 总线主令
    SYS_EVT_SRC_FAULT         // 硬件保护中断 / 底层异常
} Sys_Event_Source_t;

typedef struct {
    uint8_t  source;          // 事件来源: Sys_Event_Source_t
    uint8_t  id;              // 实体 ID: 键值 ID 或 遥控通道号
    uint8_t  event_type;      // 动作类型: PRESS, LEASS, LONG, Long_REP
    uint8_t  count;           // 连击次数 / 扩展计数
    uint32_t param;           // 32 位扩展载荷 (可选)
} Sys_Event_t;

// ==============================================================================
// 2. 优先级拦截管道过滤结果与函数指针定义
// ==============================================================================
typedef enum {
    EVENT_PASS_THROUGH = 0,   // 放行：本层不处理或不拦截，漏给下一优先级
    EVENT_CONSUMED     = 1    // 消费并拦截：本层已处理，就地销毁，管道熔断！
} Event_Result_t;

// 责任链过滤器处理函数原型 (栈直调，0 RAM 纳秒级响应)
typedef Event_Result_t (*Sys_Pipeline_Filter_t)(const Sys_Event_t *p_evt);

// ==============================================================================
// 3. 核心责任链分发引擎 API
// ==============================================================================

/**
 * @brief  初始化路由器引擎
 */
void Sys_Router_Init(void);

/**
 * @brief  挂载策略层静态过滤表 (由 APP 决策层在系统初始化时传入)
 * @param  p_table: 过滤器函数指针数组首地址
 * @param  count: 过滤器数量
 * @return true 挂载成功; false 失败
 */
bool Sys_Router_RegisterFilterTable(const Sys_Pipeline_Filter_t *p_table, uint8_t count);

/**
 * @brief  事件进入优先级管道自顶向下分发 (零动态内存，栈直调执行)
 * @param  p_evt: 统一事件对象指针
 * @return EVENT_CONSUMED 表示被某一优先级拦截消费；EVENT_PASS_THROUGH 表示全管道穿透
 */
Event_Result_t Sys_Router_Dispatch(const Sys_Event_t *p_evt);

// 兼容别名
#define Sys_Event_Dispatch(evt) Sys_Router_Dispatch(evt)

// ==============================================================================
// 4. 多载体交付适配接口 (Multi-Carrier Sinks)
// ==============================================================================

// --- 载体 A: 单值覆盖邮箱 (供控制任务读取运动指令，只留最新值，防积压) ---
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

void Sys_Mailbox_PostMotionCmd(uint8_t src, uint8_t id, uint8_t event_type);
bool Sys_Mailbox_GetMotionCmd(uint8_t *p_src, uint8_t *p_id, uint8_t *p_event_type);
void Sys_Mailbox_ClearMotionCmd(void);

// --- 载体 B: 任务通知 (用于菜单按键唤醒，0 额外队列 RAM 开销) ---
#if SYS_ROUTER_USE_FREERTOS
void Sys_Router_RegisterMenuTask(TaskHandle_t task_handle);
bool Sys_Router_NotifyMenuKey(uint8_t key_id, uint8_t event_type, uint8_t count);
bool Sys_Router_WaitMenuKey(uint8_t *p_key_id, uint8_t *p_event_type, uint8_t *p_count, uint32_t timeout_ms);

// 兼容别名
#define Sys_Supervisor_RegisterMenuTask Sys_Router_RegisterMenuTask
#define Sys_Supervisor_NotifyMenuKey    Sys_Router_NotifyMenuKey
#define Sys_Supervisor_WaitMenuKey      Sys_Router_WaitMenuKey
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MID_ROUTER_H */
