#include "mid_led.h"

void MID_LED_Init(void)
{
    // GPIO初始化已在main.c的MX_GPIO_Init()中完成
    // 默认关闭两个灯带输出
    MID_LED_Write(MID_LED_1, false);
    MID_LED_Write(MID_LED_2, false);
}

void MID_LED_Write(MID_LED_ID id, bool state)
{
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
    switch (id)
    {
        case MID_LED_1:
            HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
            break;
        case MID_LED_2:
            HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
            break;
        default:
            break;
    }
}
