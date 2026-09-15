#include "mid_led.h"

static bool s_led_state[MID_LED_COUNT] = {false, false};

void MID_LED_Init(void)
{
    // GPIO初始化已在main.c的MX_GPIO_Init()中完成
    // 默认关闭两个灯带输出
    MID_LED_Write(MID_LED_1, false);
    MID_LED_Write(MID_LED_2, false);
}

void MID_LED_Write(MID_LED_ID id, bool state)
{
    if (id >= MID_LED_COUNT) return;
    s_led_state[id] = state;
    GPIO_PinState pinState = state ? GPIO_PIN_RESET : GPIO_PIN_SET;
    switch (id)
    {
        case MID_LED_1:
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, pinState);
            break;
        case MID_LED_2:
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, pinState);
            break;
        default:
            break;
    }
}

void MID_LED_Toggle(MID_LED_ID id)
{
    if (id >= MID_LED_COUNT) return;
    MID_LED_Write(id, !s_led_state[id]);
}

bool MID_LED_ReadState(MID_LED_ID id)
{
    if (id >= MID_LED_COUNT) return false;
    return s_led_state[id];
}

void MID_LED_ToggleAll(void)
{
    bool is_any_on = (s_led_state[MID_LED_1] || s_led_state[MID_LED_2]);
    if (is_any_on) {
        MID_LED_Write(MID_LED_1, false);
        MID_LED_Write(MID_LED_2, false);
    } else {
        MID_LED_Write(MID_LED_1, true);
        MID_LED_Write(MID_LED_2, true);
    }
}

