#include "mid_Key.h"

static MID_KEY_HandleTypeDef mid_key[MID_KEY_COUNT];
static QueueHandle_t mid_Key_queue; /* 按键事件队列句柄 */
volatile uint8_t tm1650_raw_key = 0;

static void MID_KEY_Init(MID_KEY_HandleTypeDef *mid_key, MID_Key_ID key_id);

void MID_KEY_HardwareBinding(void)
{
    /* 1. 创建事件队列（存放单按键消息） */
    mid_Key_queue = xQueueCreate(8, sizeof(MID_KEY_SingleKeyMsg));

    MID_KEY_Init(&mid_key[MID_KEY_ID_K1], MID_KEY_ID_K1);
    MID_KEY_Init(&mid_key[MID_KEY_ID_K2], MID_KEY_ID_K2);
    MID_KEY_Init(&mid_key[MID_KEY_ID_K3], MID_KEY_ID_K3);
    MID_KEY_Init(&mid_key[MID_KEY_ID_K4], MID_KEY_ID_K4);
    MID_KEY_Init(&mid_key[MID_KEY_ID_K5], MID_KEY_ID_K5);
    MID_KEY_Init(&mid_key[MID_KEY_ID_K6], MID_KEY_ID_K6);
}

static void MID_KEY_Init(MID_KEY_HandleTypeDef *mid_key, MID_Key_ID key_id)
{
    mid_key->ID = key_id;
    mid_key->state = KEY_ST_IDLE;  /* 当前状态 */
    mid_key->debounce_cnt = 0;     /* 消抖计数器 */
    mid_key->press_start_tick = 0; /* 按下时刻系统tick (ms) */
    mid_key->long_reported = 0;    /* 是否长按已上报 */
    mid_key->press_reported = 0;   /* 是否按下已上报 */
}

static void MID_Key_ReportSingle(MID_Key_ID id, MID_KeyEventType evt)
{
    MID_KEY_SingleKeyMsg msg;
    msg.key_id = id;
    msg.event = evt;
    xQueueSend(mid_Key_queue, &msg, 0);
}

BaseType_t MID_Key_GetSingleEvent(MID_KEY_SingleKeyMsg *msg, TickType_t wait)
{
    if (mid_Key_queue == NULL) return pdFALSE;
    return xQueueReceive(mid_Key_queue, msg, wait);
}

/* ----- 单个按键的状态机扫描 ----- */
static void MID_Key_ScanSingleKey(MID_KEY_HandleTypeDef *mid_key, bool pressed, uint32_t nowMs)
{
    switch (mid_key->state)
    {
    case KEY_ST_IDLE:
        if (pressed)
        {
            mid_key->state = KEY_ST_DEBOUNCE;
            mid_key->debounce_cnt = 0;
        }
        break;

    case KEY_ST_DEBOUNCE:
        if (pressed)
        {
            mid_key->debounce_cnt++;
            if (mid_key->debounce_cnt >= MID_KEY_DEBOUNCE_TICKS)
            {
                mid_key->state = KEY_ST_PRESSED;
                mid_key->press_start_tick = nowMs;
                mid_key->long_reported = false;
                mid_key->press_reported = false;
            }
        }
        else
        {
            mid_key->state = KEY_ST_IDLE;
        }
        break;

    case KEY_ST_PRESSED:
        if (!pressed)
        {
            uint32_t duration = nowMs - mid_key->press_start_tick;
            if (duration <= MID_KEY_SHORT_MAX_TICKS * MID_KEY_SCAN_PERIOD_MS)
            {
                MID_Key_ReportSingle(mid_key->ID, MID_KEY_EVT_LEASS);
            }
            mid_key->state = KEY_ST_IDLE;
        }
        else
        {
            uint32_t duration = nowMs - mid_key->press_start_tick;
            if (duration > MID_KEY_SHORT_MAX_TICKS * MID_KEY_SCAN_PERIOD_MS)
            {
                mid_key->state = KEY_ST_LONG_WAIT;
            }

            if (!mid_key->press_reported)
            {
                mid_key->press_reported = true;
                MID_Key_ReportSingle(mid_key->ID, MID_KEY_EVT_PRESS);
            }
        }
        break;

    case KEY_ST_LONG_WAIT:
        if (!pressed)
        {
            mid_key->state = KEY_ST_IDLE;
        }
        else
        {
            uint32_t duration = nowMs - mid_key->press_start_tick;
            if (!mid_key->long_reported && duration >= MID_KEY_LONG_TICKS * MID_KEY_SCAN_PERIOD_MS)
            {
                MID_Key_ReportSingle(mid_key->ID, MID_KEY_EVT_LONG);
                mid_key->long_reported = true;
            }
            else if (mid_key->long_reported == true && duration >= MID_KEY_LONG_TICKS * MID_KEY_SCAN_PERIOD_MS)
            {
                MID_Key_ReportSingle(mid_key->ID, MID_KEY_EVT_Long_REP);
            }
        }
        break;
    }
}

static void MID_Key_ScanAll(void)
{
    uint32_t nowMs = xTaskGetTickCount();

    // 1. 调用 TM1650 的按键扫描函数
    uint8_t key_val = MID_TM1650_ReadKey();
    tm1650_raw_key = key_val;

    // 2. 解码映射为 6 个虚拟按键的状态（按下为1，未按下为0）
    // 0x54=84 (K1), 0x5c=92 (K2), 0x64=100 (K3), 0x6c=108 (K4), 0x4c=76 (K5), 0x44=68 (K6)
    bool virtual_k1 = (key_val == 84);
    bool virtual_k2 = (key_val == 92);
    bool virtual_k3 = (key_val == 100);
    bool virtual_k4 = (key_val == 108);
    bool virtual_k5 = (key_val == 76);
    bool virtual_k6 = (key_val == 68);

    // 3. 遍历所有按键，逐个更新状态机
    for (uint8_t i = 0; i < MID_KEY_COUNT; i++)
    {
        bool pressed = false;
        if (mid_key[i].ID == MID_KEY_ID_K1)      pressed = virtual_k1;
        else if (mid_key[i].ID == MID_KEY_ID_K2) pressed = virtual_k2;
        else if (mid_key[i].ID == MID_KEY_ID_K3) pressed = virtual_k3;
        else if (mid_key[i].ID == MID_KEY_ID_K4) pressed = virtual_k4;
        else if (mid_key[i].ID == MID_KEY_ID_K5) pressed = virtual_k5;
        else if (mid_key[i].ID == MID_KEY_ID_K6) pressed = virtual_k6;

        MID_Key_ScanSingleKey(&mid_key[i], pressed, nowMs);
    }
}

void MID_Key_ScanTask(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while (1)
    {
        MID_Key_ScanAll();
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(MID_KEY_SCAN_PERIOD_MS));
    }
}
