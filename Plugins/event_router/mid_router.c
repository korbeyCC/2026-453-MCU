#include "mid_router.h"
#include <stddef.h>

// ==============================================================================
// 静态私有变量
// ==============================================================================

// 过滤器表指针与长度 (由策略层在初始化时注入)
static const Sys_Pipeline_Filter_t *s_filter_table = NULL;
static uint8_t s_filter_count = 0;

// 载体 A: 单值覆盖邮箱实体 (用于电机运动主令与微调指令，防积压)
static Sys_Motion_Mailbox_t s_motion_mailbox = {SYS_MOTION_SRC_NONE, 0, 0, false};

#if SYS_ROUTER_USE_FREERTOS
// 载体 B: 任务通知句柄 (用于唤醒菜单调参任务)
static TaskHandle_t s_menu_task_handle = NULL;
#endif

// ==============================================================================
// 核心责任链引擎实现
// ==============================================================================

void Sys_Router_Init(void)
{
    s_filter_table = NULL;
    s_filter_count = 0;
    s_motion_mailbox.src        = SYS_MOTION_SRC_NONE;
    s_motion_mailbox.id         = 0;
    s_motion_mailbox.event_type = 0;
    s_motion_mailbox.has_cmd    = false;

#if SYS_ROUTER_USE_FREERTOS
    s_menu_task_handle = NULL;
#endif
}

bool Sys_Router_RegisterFilterTable(const Sys_Pipeline_Filter_t *p_table, uint8_t count)
{
    if (p_table == NULL || count == 0) {
        return false;
    }
    s_filter_table = p_table;
    s_filter_count = count;
    return true;
}

Event_Result_t Sys_Router_Dispatch(const Sys_Event_t *p_evt)
{
    if (p_evt == NULL || s_filter_table == NULL) {
        return EVENT_PASS_THROUGH;
    }

    // 顺序遍历责任链过滤器 (按优先级严格降序 Level 0 -> Level 1 -> ...)
    for (uint8_t i = 0; i < s_filter_count; i++) {
        if (s_filter_table[i] != NULL) {
            if (s_filter_table[i](p_evt) == EVENT_CONSUMED) {
                return EVENT_CONSUMED; // 拦截消费，管道立即熔断下传！
            }
        }
    }

    return EVENT_PASS_THROUGH; // 全管道放行穿透
}

// ==============================================================================
// 多载体交付适配实现
// ==============================================================================

// --- 载体 A: 单值覆盖邮箱 ---

void Sys_Mailbox_PostMotionCmd(uint8_t src, uint8_t id, uint8_t event_type)
{
#if SYS_ROUTER_USE_FREERTOS
    taskENTER_CRITICAL();
#endif
    s_motion_mailbox.src        = src;
    s_motion_mailbox.id         = id;
    s_motion_mailbox.event_type = event_type;
    s_motion_mailbox.has_cmd    = true;
#if SYS_ROUTER_USE_FREERTOS
    taskEXIT_CRITICAL();
#endif
}

bool Sys_Mailbox_GetMotionCmd(uint8_t *p_src, uint8_t *p_id, uint8_t *p_event_type)
{
    if (!s_motion_mailbox.has_cmd) return false;

#if SYS_ROUTER_USE_FREERTOS
    taskENTER_CRITICAL();
#endif
    if (p_src) *p_src               = s_motion_mailbox.src;
    if (p_id) *p_id                 = s_motion_mailbox.id;
    if (p_event_type) *p_event_type = s_motion_mailbox.event_type;
    s_motion_mailbox.has_cmd        = false; // 取走即清除
#if SYS_ROUTER_USE_FREERTOS
    taskEXIT_CRITICAL();
#endif
    return true;
}

void Sys_Mailbox_ClearMotionCmd(void)
{
#if SYS_ROUTER_USE_FREERTOS
    taskENTER_CRITICAL();
#endif
    s_motion_mailbox.has_cmd = false;
#if SYS_ROUTER_USE_FREERTOS
    taskEXIT_CRITICAL();
#endif
}

// --- 载体 B: 任务通知 (FreeRTOS 专属) ---
#if SYS_ROUTER_USE_FREERTOS

void Sys_Router_RegisterMenuTask(TaskHandle_t task_handle)
{
    s_menu_task_handle = task_handle;
}

bool Sys_Router_NotifyMenuKey(uint8_t key_id, uint8_t event_type, uint8_t count)
{
    if (s_menu_task_handle == NULL) return false;

    uint32_t val = ((uint32_t)key_id) | (((uint32_t)event_type) << 8) | (((uint32_t)count) << 16);
    return (xTaskNotify(s_menu_task_handle, val, eSetValueWithOverwrite) == pdPASS);
}

bool Sys_Router_WaitMenuKey(uint8_t *p_key_id, uint8_t *p_event_type, uint8_t *p_count, uint32_t timeout_ms)
{
    uint32_t notify_val = 0;
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &notify_val, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        if (p_key_id) *p_key_id = (uint8_t)(notify_val & 0xFF);
        if (p_event_type) *p_event_type = (uint8_t)((notify_val >> 8) & 0xFF);
        if (p_count) *p_count = (uint8_t)((notify_val >> 16) & 0xFF);
        return true;
    }
    return false;
}

#endif
