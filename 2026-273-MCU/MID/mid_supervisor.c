#include "mid_supervisor.h"
#include "app_control.h"
#include "app_Data.h"
#include "mid_signal.h"
#include "mid_Key.h"
#include <math.h>
#include <stdio.h>

// ==============================================================================
// 全局状态与多载体容器实体
// ==============================================================================
static volatile System_Mode_t s_sys_mode = SYS_MODE_STANDBY;

// 载体 A: 单值覆盖邮箱 (用于电机运动主令与微调指令，防积压)
static Sys_Motion_Mailbox_t s_motion_mailbox = {0, 0, 0, false};

// 状态: 单轴微调方向 (0: 上升/正转 -UP-, 1: 下降/反转 -dn-)
static volatile uint8_t s_tune_dir = 0;

#if SYS_SUPERVISOR_USE_FREERTOS
// 载体 B: 任务通知句柄 (用于唤醒菜单调参任务)
static TaskHandle_t s_menu_task_handle = NULL;
#endif

// ==============================================================================
// 管道过滤器定义 (按优先级严格降序: Level 0 -> Level 1 -> Level 2 -> Level 3)
// ==============================================================================

/**
 * @brief 【Level 0】紧急安全层拦截器 (Priority 0: 最高安全特权)
 * @details 判定急停遥控 (A 键)，直接执行 Direct Action 切断输出，绝不走异步排队
 */
static Event_Result_t Filter_Level0_Safety(const Sys_Event_t *p_evt)
{
    if (p_evt == NULL) return EVENT_PASS_THROUGH;

    // 遥控器 A 键 (MID_SIGNAL_REMOT_2): 无论何时按下均作为急停/制动指令无条件拦截
    if (p_evt->source == SYS_EVT_SRC_SIGNAL && p_evt->id == MID_SIGNAL_REMOT_2) {
        if (p_evt->event_type == MID_SIGNAL_EVT_TRIGGER || p_evt->event_type == MID_SIGNAL_EVT_LONG) {
            APP_Control_EmergencyStop();
            Debug_Printf("[LEPA Level 0] E-Stop Intercepted via Remote A Key (Direct Action)!\r\n");
            return EVENT_CONSUMED; // 就地消费，终止下传
        }
    }

    return EVENT_PASS_THROUGH; // 无安全威胁，放行给下一层
}

/**
 * @brief 【Level 1】故障与消警层拦截器 (Priority 1: 故障处理特权)
 * @details 当系统处于严重故障闪烁锁死状态时，任何按键仅作为消警/复位响应，杜绝误动电机
 */
static Event_Result_t Filter_Level1_FaultClear(const Sys_Event_t *p_evt)
{
    if (p_evt == NULL) return EVENT_PASS_THROUGH;

    if (Sys_Mode_IsFaultLocked()) {
        // 如果是触发类或长按类事件，尝试执行故障清除
        if (p_evt->event_type == MID_KEY_EVT_LEASS || p_evt->event_type == MID_SIGNAL_EVT_TRIGGER ||
            p_evt->event_type == MID_KEY_EVT_LONG || p_evt->event_type == MID_SIGNAL_EVT_LONG) {
            if (APP_Control_ClearFault()) {
                Debug_Printf("[LEPA Level 1] Fault Cleared via Input Event (Consumed)!\r\n");
            } else {
                Debug_Printf("[LEPA Level 1] Waiting for Driver Recovery before Clear (Consumed)!\r\n");
            }
            return EVENT_CONSUMED; // 故障态下按键被消警捕获，绝不下传
        }
        return EVENT_CONSUMED; // 故障态下其余事件直接静默吞掉
    }

    return EVENT_PASS_THROUGH;
}

/**
 * @brief 【Level 2】前台交互焦点与执行自治路由层 (Priority 2: 核心分流中枢)
 * @details 
 *   1. 调参菜单中: K1~K6 全部路由给菜单任务独占，屏蔽遥控；
 *   2. 运动控制中: 任意按键作为打断意图直接送入控制任务邮箱；
 *   3. 待机就绪态: K1~K4 (动作键) 路由给控制任务自启微调；K5~K6 (功能键) 路由给菜单任务处理
 */
static Event_Result_t Filter_Level2_MenuFocus(const Sys_Event_t *p_evt)
{
    if (p_evt == NULL) return EVENT_PASS_THROUGH;

    // 1. 场景一：调参菜单态 (SYS_MODE_MENU_CONFIG)
    if (Sys_Mode_IsMenuActive()) {
        if (p_evt->source == SYS_EVT_SRC_KEY) {
#if SYS_SUPERVISOR_USE_FREERTOS
            Sys_Supervisor_NotifyMenuKey(p_evt->id, p_evt->event_type, p_evt->count);
#endif
            return EVENT_CONSUMED;
        }

        if (p_evt->source == SYS_EVT_SRC_SIGNAL) {
            Debug_Printf("[LEPA Level 2] Menu Config Active: Blocked Remote Motion Signal %d (Safety Gate)!\r\n", p_evt->id);
            return EVENT_CONSUMED; // 安全闭锁，阻断遥控
        }
    }

    // 2. 场景二：电机处于运动中 (单轴微调、四轴联动升降、防夹反弹、自调平)
    if (g_sys_context.system_step == SYS_STEP_SINGLE_TUNE ||
        g_sys_context.system_step == SYS_STEP_TOTAL_RUNNING ||
        g_sys_context.system_step == SYS_STEP_TOTAL_REBOUND ||
        g_sys_context.system_step == SYS_STEP_AUTO_ALIGN) {
        if (p_evt->source == SYS_EVT_SRC_KEY) {
            // 第一个按键 (K1 或 K6) 短按：放行给菜单任务，用于运行时随时切换查看各电机高度！
            if ((p_evt->id == MID_KEY_ID_K1 || p_evt->id == MID_KEY_ID_K6) && p_evt->event_type == MID_KEY_EVT_LEASS) {
#if SYS_SUPERVISOR_USE_FREERTOS
                Sys_Supervisor_NotifyMenuKey(p_evt->id, p_evt->event_type, p_evt->count);
#endif
                return EVENT_CONSUMED; // 放行给菜单消费，不打断电机运行
            }

            // 其他物理面板按键或长按：作为打断意图直接送入控制任务邮箱
            if (p_evt->event_type == MID_KEY_EVT_LEASS || p_evt->event_type == MID_KEY_EVT_LONG || p_evt->event_type == MID_KEY_EVT_Long_REP) {
                Sys_Mailbox_PostMotionCmd(SYS_MOTION_SRC_KEY, p_evt->id, p_evt->event_type);
                return EVENT_CONSUMED;
            }
        }
    }

    // 3. 场景三：待机主界面态 (SYS_MODE_STANDBY 或 SYS_MODE_DEBUG_CALIB)
    if (p_evt->source == SYS_EVT_SRC_KEY) {
        // A. K1 ~ K4: 4 轴微调动作键 -> 直接路由给控制任务，由其在 SYS_STEP_READY 内自主启动微调！
        if (p_evt->id <= MID_KEY_ID_K4 && s_sys_mode == SYS_MODE_STANDBY) {
            if (p_evt->event_type == MID_KEY_EVT_LEASS) {
                Sys_Mailbox_PostMotionCmd(SYS_MOTION_SRC_KEY, p_evt->id, p_evt->event_type);
                return EVENT_CONSUMED;
            }
        }

        // B. K5 (方向切换), K6 (轮播/长按进菜单), 及调试层按键 -> 路由给菜单任务
#if SYS_SUPERVISOR_USE_FREERTOS
        Sys_Supervisor_NotifyMenuKey(p_evt->id, p_evt->event_type, p_evt->count);
#endif
        return EVENT_CONSUMED;
    }

    return EVENT_PASS_THROUGH;
}

/**
 * @brief 【Level 3】常规业务运动层拦截器 (Priority 3: 默认兜底)
 * @details 漏过前三层的合法遥控信号，送入单值覆盖邮箱 (Mailbox)，杜绝积压
 */
static Event_Result_t Filter_Level3_Motion(const Sys_Event_t *p_evt)
{
    if (p_evt == NULL) return EVENT_PASS_THROUGH;

    if (p_evt->source == SYS_EVT_SRC_SIGNAL) {
        if (Sys_Mode_CanRunMotion()) {
            Sys_Mailbox_PostMotionCmd(SYS_MOTION_SRC_SIGNAL, p_evt->id, p_evt->event_type);
            return EVENT_CONSUMED;
        } else {
            Debug_Printf("[LEPA Level 3] Motion Request Denied by Supervisor Gate!\r\n");
            return EVENT_CONSUMED;
        }
    }

    return EVENT_PASS_THROUGH;
}

// ==============================================================================
// 静态责任链管道配置表 (零动态内存，编译期确定，完全可静态追溯)
// ==============================================================================
static const Sys_Pipeline_Filter_t c_event_pipeline[] = {
    Filter_Level0_Safety,     // Level 0: 急停与极限熔断
    Filter_Level1_FaultClear, // Level 1: 故障与报警确认
    Filter_Level2_MenuFocus,  // Level 2: 菜单独占与动作/功能分流
    Filter_Level3_Motion      // Level 3: 遥控运动单值邮箱投递
};

// ==============================================================================
// 核心 API 实现
// ==============================================================================

void Sys_Supervisor_Init(void)
{
    s_sys_mode = SYS_MODE_STANDBY;
    s_motion_mailbox.has_cmd = false;
    s_tune_dir = 0;
#if SYS_SUPERVISOR_USE_FREERTOS
    s_menu_task_handle = NULL;
#endif
    Debug_Printf("[LEPA] Supervisor & Pipeline Initialized Successfully.\r\n");
}

Event_Result_t Sys_Event_Dispatch(const Sys_Event_t *p_evt)
{
    if (p_evt == NULL) return EVENT_PASS_THROUGH;

    for (uint8_t i = 0; i < (sizeof(c_event_pipeline) / sizeof(c_event_pipeline[0])); i++) {
        if (c_event_pipeline[i](p_evt) == EVENT_CONSUMED) {
            return EVENT_CONSUMED;
        }
    }

    return EVENT_PASS_THROUGH;
}

void Sys_Mode_Set(System_Mode_t new_mode)
{
    if (s_sys_mode != new_mode) {
        s_sys_mode = new_mode;
        Debug_Printf("[LEPA] System Mode Transition -> %d\r\n", new_mode);
    }
}

System_Mode_t Sys_Mode_Get(void)
{
    return s_sys_mode;
}

bool Sys_Mode_CanRunMotion(void)
{
    if (s_sys_mode == SYS_MODE_MENU_CONFIG || s_sys_mode == SYS_MODE_FAULT_LOCKED) {
        return false;
    }
    if (g_sys_context.system_step == SYS_STEP_FAULT_STOP) {
        return false;
    }
    return true;
}

bool Sys_Mode_CanEnterMenu(void)
{
    if (g_sys_context.system_step == SYS_STEP_TOTAL_RUNNING ||
        g_sys_context.system_step == SYS_STEP_AUTO_ALIGN ||
        g_sys_context.system_step == SYS_STEP_SINGLE_TUNE ||
        g_sys_context.system_step == SYS_STEP_TOTAL_REBOUND) {
        return false;
    }
    return true;
}

bool Sys_Mode_IsMenuActive(void)
{
    return (s_sys_mode == SYS_MODE_MENU_CONFIG);
}

bool Sys_Mode_IsFaultLocked(void)
{
    return (s_sys_mode == SYS_MODE_FAULT_LOCKED || g_sys_context.system_step == SYS_STEP_FAULT_STOP);
}

// ==============================================================================
// 多载体适配接口实现
// ==============================================================================

// --- 载体 A: 单值覆盖邮箱 ---
void Sys_Mailbox_PostMotionCmd(uint8_t src, uint8_t id, uint8_t event_type)
{
#if SYS_SUPERVISOR_USE_FREERTOS
    taskENTER_CRITICAL();
#endif
    s_motion_mailbox.src        = src;
    s_motion_mailbox.id         = id;
    s_motion_mailbox.event_type = event_type;
    s_motion_mailbox.has_cmd    = true;
#if SYS_SUPERVISOR_USE_FREERTOS
    taskEXIT_CRITICAL();
#endif
}

bool Sys_Mailbox_GetMotionCmd(uint8_t *p_src, uint8_t *p_id, uint8_t *p_event_type)
{
    if (!s_motion_mailbox.has_cmd) return false;

#if SYS_SUPERVISOR_USE_FREERTOS
    taskENTER_CRITICAL();
#endif
    if (p_src) *p_src               = s_motion_mailbox.src;
    if (p_id) *p_id                 = s_motion_mailbox.id;
    if (p_event_type) *p_event_type = s_motion_mailbox.event_type;
    s_motion_mailbox.has_cmd        = false; // 取走即清除
#if SYS_SUPERVISOR_USE_FREERTOS
    taskEXIT_CRITICAL();
#endif
    return true;
}

void Sys_Mailbox_ClearMotionCmd(void)
{
#if SYS_SUPERVISOR_USE_FREERTOS
    taskENTER_CRITICAL();
#endif
    s_motion_mailbox.has_cmd = false;
#if SYS_SUPERVISOR_USE_FREERTOS
    taskEXIT_CRITICAL();
#endif
}

// --- 载体 B: 任务通知 (FreeRTOS 专属) ---
#if SYS_SUPERVISOR_USE_FREERTOS

void Sys_Supervisor_RegisterMenuTask(TaskHandle_t task_handle)
{
    s_menu_task_handle = task_handle;
}

bool Sys_Supervisor_NotifyMenuKey(uint8_t key_id, uint8_t event_type, uint8_t count)
{
    if (s_menu_task_handle == NULL) return false;

    uint32_t val = ((uint32_t)key_id) | (((uint32_t)event_type) << 8) | (((uint32_t)count) << 16);
    return (xTaskNotify(s_menu_task_handle, val, eSetValueWithOverwrite) == pdPASS);
}

bool Sys_Supervisor_WaitMenuKey(uint8_t *p_key_id, uint8_t *p_event_type, uint8_t *p_count, uint32_t timeout_ms)
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

// ==============================================================================
// 8. 系统只读视图模型接口实现 (View Model)
// ==============================================================================

int32_t Sys_View_GetAxisTravelMm(uint8_t axis_idx)
{
    if (axis_idx >= 4) return 0;

    float abs_mm = 0.0f;
    if (g_sys_context.counts_per_mm > 0.0f) {
        abs_mm = (float)g_sys_context.g_motor_status[axis_idx].current_abs_hall / g_sys_context.counts_per_mm;
    }

    int32_t val_mm = (int32_t)roundf(abs_mm);
    if (val_mm < 0) val_mm = 0;
    if (val_mm > 9999) val_mm = 9999;
    return val_mm;
}

int32_t Sys_View_GetAxisMountMm(uint8_t axis_idx)
{
    if (axis_idx >= 4) return 0;

    float mount_mm = 0.0f;
    if (g_sys_context.counts_per_mm > 0.0f) {
        mount_mm = (float)app_data.min_mount_halls[axis_idx] / g_sys_context.counts_per_mm;
    }

    int32_t val_mm = (int32_t)roundf(mount_mm);
    if (val_mm < 0) val_mm = 0;
    if (val_mm > 9999) val_mm = 9999;
    return val_mm;
}

uint8_t Sys_View_GetFaultCode(void)
{
    return g_sys_context.system_fault_code;
}

bool Sys_View_IsRebounding(void)
{
    return (g_sys_context.system_step == SYS_STEP_TOTAL_REBOUND);
}

Sys_Indicator_State_t Sys_View_GetIndicatorState(void)
{
    if (g_sys_context.system_step == SYS_STEP_FAULT_STOP || g_sys_context.system_fault_code != FAULT_CODE_NONE) {
        return SYS_IND_FAULT;
    } else if (g_sys_context.system_step == SYS_STEP_TOTAL_RUNNING ||
               g_sys_context.system_step == SYS_STEP_SINGLE_TUNE ||
               g_sys_context.system_step == SYS_STEP_TOTAL_REBOUND ||
               g_sys_context.system_step == SYS_STEP_AUTO_ALIGN) {
        return SYS_IND_RUNNING;
    } else {
        return SYS_IND_STEADY;
    }
}

void Sys_View_SetTuneDir(uint8_t dir)
{
    s_tune_dir = dir;
}

uint8_t Sys_View_GetTuneDir(void)
{
    return s_tune_dir;
}

// ==============================================================================
// 9. 中立通知与请求接口实现
// ==============================================================================

void Sys_Notify_ParamsUpdated(void)
{
    APP_Control_UpdateParamsFromAppData();
}

void Sys_Notify_FactoryReset(void)
{
    APP_Control_ResetSystemContext();
}

bool Sys_Request_ClearFault(void)
{
    return APP_Control_ClearFault();
}
