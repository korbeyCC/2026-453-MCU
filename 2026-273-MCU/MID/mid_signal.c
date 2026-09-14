#include "mid_signal.h"
#include "mid_supervisor.h"
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
            return (HAL_GPIO_ReadPin(REMOT_1_GPIO_Port, REMOT_1_Pin) == GPIO_PIN_SET); // 高电平有效

        case MID_SIGNAL_REMOT_2:
            return (HAL_GPIO_ReadPin(REMOT_2_GPIO_Port, REMOT_2_Pin) == GPIO_PIN_SET); // 高电平有效

        case MID_SIGNAL_REMOT_3:
            return (HAL_GPIO_ReadPin(REMOT_3_GPIO_Port, REMOT_3_Pin) == GPIO_PIN_SET); // 高电平有效

        case MID_SIGNAL_REMOT_4:
            return (HAL_GPIO_ReadPin(REMOT_4_GPIO_Port, REMOT_4_Pin) == GPIO_PIN_SET); // 高电平有效

        case MID_SIGNAL_REMOT_5:
            return (HAL_GPIO_ReadPin(REMOT_5_GPIO_Port, REMOT_5_Pin) == GPIO_PIN_SET); // 高电平有效

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

    if (Sys_Event_Dispatch(&sys_evt) == EVENT_CONSUMED) {
        return; // 被高优先级拦截消费，终止下发至队列
    }

    // 2. 兜底兼容流入旧队列
    MID_SIGNAL_Msg msg;
    msg.signal_id = id;
    msg.event     = evt;
    xQueueSend(mid_Signal_queue, &msg, 0);
}

// 单路信号消抖状态机扫描（完全参照 mid_key 的细分状态机结构）
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
                }
            } else {
                sig->state = SIGNAL_ST_IDLE;
            }
            break;

        case SIGNAL_ST_ACTIVE:
            if (!is_phys_active) {
                uint32_t duration = nowMs - sig->active_start_tick;
                if (duration <= MID_SIGNAL_SHORT_MAX_TICKS * MID_SIGNAL_SCAN_PERIOD_MS) {
                    MID_Signal_Report(sig->ID, MID_SIGNAL_EVT_RELEASE);
                }
                sig->state = SIGNAL_ST_IDLE;
            } else {
                uint32_t duration = nowMs - sig->active_start_tick;
                if (duration > MID_SIGNAL_SHORT_MAX_TICKS * MID_SIGNAL_SCAN_PERIOD_MS) {
                    sig->state = SIGNAL_ST_LONG_WAIT;
                }

                if (!sig->active_reported) {
                    sig->active_reported = true;
                    MID_Signal_Report(sig->ID, MID_SIGNAL_EVT_TRIGGER);
                }
            }
            break;

        case SIGNAL_ST_LONG_WAIT:
            if (!is_phys_active) {
                MID_Signal_Report(sig->ID, MID_SIGNAL_EVT_RELEASE);
                sig->state = SIGNAL_ST_IDLE;
            } else {
                uint32_t duration = nowMs - sig->active_start_tick;
                if (!sig->long_reported && duration >= MID_SIGNAL_LONG_TICKS * MID_SIGNAL_SCAN_PERIOD_MS) {
                    MID_Signal_Report(sig->ID, MID_SIGNAL_EVT_LONG);
                    sig->long_reported = true;
                } else if (sig->long_reported && duration >= MID_SIGNAL_LONG_TICKS * MID_SIGNAL_SCAN_PERIOD_MS) {
                    // 长按期间以 100ms 为周期上报重复触发事件，便于应用层进行持续增减调节
                    // 由于 20ms 执行一次，每隔 5 次触发上报一个 LONG_REP
                    if ((duration % 100) < MID_SIGNAL_SCAN_PERIOD_MS) {
                        MID_Signal_Report(sig->ID, MID_SIGNAL_EVT_LONG_REP);
                    }
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
