#ifndef __MID_KEY_H
#define __MID_KEY_H

#include "mid_main.h"
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"

/* ---------------------------------- 按键ID ---------------------------------- */
typedef enum {
    MID_KEY_ID_K1, // 对应按键1 (A)
    MID_KEY_ID_K2, // 对应按键2 (B)
    MID_KEY_ID_K3, // 对应按键3 (C)
    MID_KEY_ID_K4, // 对应按键4 (D)
    MID_KEY_ID_K5, // 对应按键5 (方向)
    MID_KEY_ID_K6, // 对应按键6 (设置)
    // 最后一个，用于计数
    MID_KEY_COUNT,
} MID_Key_ID;

/* ----- 单按键状态机状态枚举 ----- */
typedef enum {
    KEY_ST_IDLE,     /* 空闲状态，无按键按下 */
    KEY_ST_DEBOUNCE, /* 消抖状态，检测到电平变化，等待稳定 */
    KEY_ST_PRESSED,  /* 已确认按下，处于短按有效期内 */
    KEY_ST_LONG_WAIT /* 超过短按最大时长，进入长按等待区 */
} MID_KeyState;

/* 每个按键的控制块，保存状态机变量 */
typedef struct
{
    MID_Key_ID ID;             /* 按键 ID */
    MID_KeyState state;        /* 当前状态 */
    uint8_t debounce_cnt;      /* 消抖计数器 */
    uint32_t press_start_tick; /* 按下时刻的系统tick (ms) */
    bool long_reported;        /* 是否已上报长按 */
    bool press_reported;       /* 是否已上报按下 */
} MID_KEY_HandleTypeDef;

/* ========== 时间参数（单位：扫描周期）========== */
#define MID_KEY_SCAN_PERIOD_MS  20 /* 按键扫描任务的周期，单位ms */
#define MID_KEY_DEBOUNCE_TICKS  2  /* 消抖所需连续次数（2*20=40ms消抖） */
#define MID_KEY_LONG_TICKS      40 /* 长按判定阈值（40*20=800ms） */
#define MID_KEY_REPEAT_TICKS    8  /* 连发间隔（8*20=160ms） */

/* ========== 事件类型 ========== */
typedef enum {
    MID_KEY_EVT_NONE = 0, /* 无事件 */
    MID_KEY_EVT_PRESS,    /* 按下事件 */
    MID_KEY_EVT_LEASS,    /* 松开/短按事件 */
    MID_KEY_EVT_LONG,     /* 长按事件 */
    MID_KEY_EVT_Long_REP, /* 长按重复事件 */
} MID_KeyEventType;

/* 单按键事件消息结构体（通过队列发送给应用层） */
typedef struct
{
    MID_Key_ID key_id;      /* 按键ID */
    MID_KeyEventType event; /* 按键事件 */
} MID_KEY_SingleKeyMsg;

void MID_KEY_HardwareBinding(void);
void MID_Key_ScanTask(void *pvParameters);
BaseType_t MID_Key_GetSingleEvent(MID_KEY_SingleKeyMsg *msg, TickType_t wait);

#endif
