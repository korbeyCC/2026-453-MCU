#include "mid_signal.h"
#include "mid_router.h"
#include "task.h"

static MID_SIGNAL_HandleTypeDef mid_signals[MID_SIGNAL_COUNT];
static QueueHandle_t mid_Signal_queue; /* 信号事件队列句柄 */

static void MID_Signal_Init(MID_SIGNAL_HandleTypeDef *sig, MID_Signal_ID id)
{
    sig->ID                = id;
    sig->state             = SIGNAL_ST_IDLE;
    sig->debounce_cnt      = 0;
    sig->active_start_tick = 0;
    sig->long_reported     = false;
    sig->active_reported   = false;
    sig->hold_5s_reported  = false;
    sig->release_cnt       = 0;
}

void MID_SIGNAL_HardwareBinding(void)
{
    // 创建事件队列，最多缓存 8 个信号消息
    mid_Signal_queue = xQueueCreate(8, sizeof(MID_SIGNAL_Msg));

    for (uint8_t i = 0; i < MID_SIGNAL_COUNT; i++) {
        MID_Signal_Init(&mid_signals[i], (MID_Signal_ID)i);
    }
}

BaseType_t MID_Signal_GetEvent(MID_SIGNAL_Msg *msg, TickType_t wait)
{
    if (mid_Signal_queue == NULL) return pdFALSE;
    return xQueueReceive(mid_Signal_queue, msg, wait);
}

// 物理引脚电平读取函数（使用 CubeMX 生成的 User Label 宏）
static bool MID_Signal_ReadPhysState(MID_Signal_ID id)
{
    switch (id) {
        case MID_SIGNAL_REMOT_1:
            return (HAL_GPIO_ReadPin(REMOT_4_GPIO_Port, REMOT_4_Pin) == GPIO_PIN_SET); // 高电平有效

        case MID_SIGNAL_REMOT_2:
            return (HAL_GPIO_ReadPin(REMOT_3_GPIO_Port, REMOT_3_Pin) == GPIO_PIN_SET); // 高电平有效

        case MID_SIGNAL_REMOT_3:
            return (HAL_GPIO_ReadPin(REMOT_2_GPIO_Port, REMOT_2_Pin) == GPIO_PIN_SET); // 高电平有效

        case MID_SIGNAL_REMOT_4:
            return (HAL_GPIO_ReadPin(REMOT_1_GPIO_Port, REMOT_1_Pin) == GPIO_PIN_SET); // 高电平有效

            /*因为换了遥控器，把引脚号也换一下*/

        case MID_SIGNAL_REMOT_5:
            return 0; // 屏蔽不需要的//(HAL_GPIO_ReadPin(REMOT_5_GPIO_Port, REMOT_5_Pin) == GPIO_PIN_SET); // 高电平有效

        case MID_SIGNAL_BUTON_DW:
            return (HAL_GPIO_ReadPin(BUTON_DW_GPIO_Port, BUTON_DW_Pin) == GPIO_PIN_RESET); // 光耦低电平有效

        case MID_SIGNAL_BUTON_UP:
            return (HAL_GPIO_ReadPin(BUTON_UP_GPIO_Port, BUTON_UP_Pin) == GPIO_PIN_RESET); // 光耦低电平有效

        default:
            return false;
    }
}

static void MID_Signal_Report(MID_Signal_ID id, MID_Signal_EventType evt)
{
    // 1. 优先送入 LEPA 优先级管道进行栈直调拦截 (Level 0 急停、Level 1 消警、Level 2 菜单独占拦截)
    Sys_Event_t sys_evt;
    sys_evt.source     = SYS_EVT_SRC_SIGNAL;
    sys_evt.id         = (uint8_t)id;
    sys_evt.event_type = (uint8_t)evt;
    sys_evt.count      = 0;
    sys_evt.param      = 0;

    if (Sys_Router_Dispatch(&sys_evt) == EVENT_CONSUMED) {
        return; // 被高优先级拦截消费，终止下发至队列
    }

    // 2. 兜底兼容流入旧队列
    MID_SIGNAL_Msg msg;
    msg.signal_id = id;
    msg.event     = evt;
    xQueueSend(mid_Signal_queue, &msg, 0);
}

// 单路信号消抖状态机扫描 (支持 150ms 释放滤波防空中掉帧、短按松手触发、按住满 5 秒超长按触发)
static void MID_Signal_ScanSingle(MID_SIGNAL_HandleTypeDef *sig, uint32_t nowMs)
{
    bool is_phys_active = MID_Signal_ReadPhysState(sig->ID);

    switch (sig->state) {
        case SIGNAL_ST_IDLE:
            if (is_phys_active) {
                sig->state        = SIGNAL_ST_DEBOUNCE;
                sig->debounce_cnt = 0;
            }
            break;

        case SIGNAL_ST_DEBOUNCE:
            if (is_phys_active) {
                sig->debounce_cnt++;
                if (sig->debounce_cnt >= MID_SIGNAL_DEBOUNCE_TICKS) {
                    sig->state             = SIGNAL_ST_ACTIVE;
                    sig->active_start_tick = nowMs;
                    sig->long_reported     = false;
                    sig->active_reported   = false;
                    sig->hold_5s_reported  = false;
                    sig->release_cnt       = 0;
                }
            } else {
                sig->state = SIGNAL_ST_IDLE;
            }
            break;

        case SIGNAL_ST_ACTIVE:
        case SIGNAL_ST_LONG_WAIT:
            if (is_phys_active) {
                sig->release_cnt = 0; // 只要引脚高电平就重置释放计数
                uint32_t duration = nowMs - sig->active_start_tick;

                // 首次按下触发通知 (用于急停 C 键与照明 D 键即时生效)
                if (!sig->active_reported) {
                    sig->active_reported = true;
                    MID_Signal_Report(sig->ID, MID_SIGNAL_EVT_TRIGGER);
                }

                // 常规长按 (1.0s) 通知
                if (!sig->long_reported && duration >= (MID_SIGNAL_LONG_TICKS * MID_SIGNAL_SCAN_PERIOD_MS)) {
                    sig->long_reported = true;
                    MID_Signal_Report(sig->ID, MID_SIGNAL_EVT_LONG);
                }

                // 超长按 5.0 秒 (5000ms) 通知 (启动整体持续点动微调)
                if (!sig->hold_5s_reported && duration >= MID_SIGNAL_HOLD_5S_MS) {
                    sig->hold_5s_reported = true;
                    MID_Signal_Report(sig->ID, MID_SIGNAL_EVT_HOLD_5S);
                }

                if (duration > (MID_SIGNAL_SHORT_MAX_TICKS * MID_SIGNAL_SCAN_PERIOD_MS)) {
                    sig->state = SIGNAL_ST_LONG_WAIT;
                }
            } else {
                // 引脚低电平：执行 150ms 释放滤波 (防 433MHz 空中瞬时掉帧)
                sig->release_cnt++;
                if (sig->release_cnt >= MID_SIGNAL_RELEASE_FILTER_TICKS) {
                    uint32_t press_duration = nowMs - sig->active_start_tick;

                    if (sig->hold_5s_reported) {
                        // 已经激活过 5 秒超长按持续微调 -> 确认松手，上报 RELEASE 触发刹停
                        MID_Signal_Report(sig->ID, MID_SIGNAL_EVT_RELEASE);
                    } else if (press_duration < (MID_SIGNAL_SHORT_MAX_TICKS * MID_SIGNAL_SCAN_PERIOD_MS)) {
                        // 按住时间小于 1.0 秒即松开 -> 确认为普通短按松手，上报 SHORT_RELEASE 触发常规自锁升降
                        MID_Signal_Report(sig->ID, MID_SIGNAL_EVT_SHORT_RELEASE);
                    } else {
                        // 按住 1.0 秒 ~ 5.0 秒之间松开 -> 判定为放弃或误触，静默回到空闲，不动作
                    }

                    sig->state = SIGNAL_ST_IDLE;
                }
            }
            break;

        default:
            sig->state = SIGNAL_ST_IDLE;
            break;
    }
}

static void MID_Signal_ScanAll(void)
{
    uint32_t nowMs = xTaskGetTickCount();
    for (uint8_t i = 0; i < MID_SIGNAL_COUNT; i++) {
        MID_Signal_ScanSingle(&mid_signals[i], nowMs);
    }
}

void MID_Signal_ScanTask(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while (1) {
        MID_Signal_ScanAll();
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(MID_SIGNAL_SCAN_PERIOD_MS));
    }
}

bool MID_Signal_GetState(MID_Signal_ID id)
{
    if (id >= MID_SIGNAL_COUNT) return false;
    return (mid_signals[id].state == SIGNAL_ST_ACTIVE || mid_signals[id].state == SIGNAL_ST_LONG_WAIT);
}
