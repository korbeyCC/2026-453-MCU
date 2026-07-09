#ifndef __MID_SIGNAL_H
#define __MID_SIGNAL_H

#include "mid_main.h"
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"

/* -------------------------------- 外接信号ID -------------------------------- */
typedef enum {
    MID_SIGNAL_REMOT_1 = 0, // PA5, 遥控按键 1 (REMOT_1_Pin)
    MID_SIGNAL_REMOT_2,     // PA6, 遥控按键 2 (REMOT_2_Pin)
    MID_SIGNAL_REMOT_3,     // PA7, 遥控按键 3 (REMOT_3_Pin)
    MID_SIGNAL_REMOT_4,     // PC4, 遥控按键 4 (REMOT_4_Pin)
    MID_SIGNAL_REMOT_5,     // PC5, 遥控按键 5 (REMOT_5_Pin)
    MID_SIGNAL_BUTON_DW,    // PC7, 外接信号下 (BUTON_DW_Pin)
    MID_SIGNAL_BUTON_UP,    // PC8, 外接信号上 (BUTON_UP_Pin)
    MID_SIGNAL_COUNT,       // 计数
} MID_Signal_ID;

/* ----- 单信号状态机状态枚举 ----- */
typedef enum {
    SIGNAL_ST_IDLE,     /* 空闲，无信号有效 */
    SIGNAL_ST_DEBOUNCE, /* 消抖状态，等待稳定 */
    SIGNAL_ST_ACTIVE,   /* 确认触发有效，相当于短触发 */
    SIGNAL_ST_LONG_WAIT /* 持续有效/长触发判定区 */
} MID_SignalState;

/* 信号控制块，保存状态机变量 */
typedef struct
{
    MID_Signal_ID ID;
    MID_SignalState state;      /* 状态 */
    uint8_t debounce_cnt;       /* 消抖计数器 */
    uint32_t active_start_tick; /* 触发时刻的系统tick (ms) */
    bool long_reported;         /* 是否已上报长触发 */
    bool active_reported;       /* 是否已上报触发 */
} MID_SIGNAL_HandleTypeDef;

/* ========== 时间参数 ========== */
#define MID_SIGNAL_SCAN_PERIOD_MS  5  // 信号轮询扫描周期ms
#define MID_SIGNAL_DEBOUNCE_TICKS  2  // 消抖所需连续次数 (2 * 20 = 40ms)
#define MID_SIGNAL_SHORT_MAX_TICKS 40 // 短周期上限 (40 * 20 = 800ms)
#define MID_SIGNAL_LONG_TICKS      50 // 长周期判定阈值 (50 * 20 = 1000ms)

/* ========== 事件类型 ========== */
typedef enum {
    MID_SIGNAL_EVT_NONE = 0,
    MID_SIGNAL_EVT_TRIGGER, // 信号检测到触发生效
    MID_SIGNAL_EVT_RELEASE, // 信号撤销释放
    MID_SIGNAL_EVT_LONG,    // 持续长触发生效
    MID_SIGNAL_EVT_LONG_REP // 持续长触发重复事件
} MID_Signal_EventType;

/* 信号队列传输消息结构体 */
typedef struct
{
    MID_Signal_ID signal_id;    // 信号 ID
    MID_Signal_EventType event; // 事件类型
} MID_SIGNAL_Msg;

void MID_SIGNAL_HardwareBinding(void);
void MID_Signal_ScanTask(void *pvParameters);
BaseType_t MID_Signal_GetEvent(MID_SIGNAL_Msg *msg, TickType_t wait);
bool MID_Signal_GetState(MID_Signal_ID id);

#endif
