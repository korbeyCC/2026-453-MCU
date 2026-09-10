#include "mid_run_led.h"
#include "main.h"

static run_led_mode_t s_current_mode = RUN_LED_MODE_STEADY_ON;
static uint16_t s_timer_ms = 0;
static bool s_led_state = true; // true: 亮, false: 灭

/**
 * @brief 工作指示灯初始化 (PA12, 待机默认常亮)
 */
void MID_RunLED_Init(void)
{
    s_current_mode = RUN_LED_MODE_STEADY_ON;
    s_timer_ms = 0;
    s_led_state = true;
    MID_RunLED_Write(true);
}

/**
 * @brief 控制指示灯开/关
 * @param on true: 点亮 (PA12 拉低), false: 熄灭 (PA12 拉高)
 */
void MID_RunLED_Write(bool on)
{
    s_led_state = on;
    // PA12 灌电流驱动: 低电平(RESET)点亮, 高电平(SET)熄灭
    HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief 翻转指示灯状态
 */
void MID_RunLED_Toggle(void)
{
    s_led_state = !s_led_state;
    HAL_GPIO_TogglePin(RUN_LED_GPIO_Port, RUN_LED_Pin);
}

/**
 * @brief 切换指示灯工作模式
 */
void MID_RunLED_SetMode(run_led_mode_t mode)
{
    if (s_current_mode != mode) {
        s_current_mode = mode;
        s_timer_ms = 0;
        if (mode == RUN_LED_MODE_STEADY_ON) {
            MID_RunLED_Write(true);
        } else if (mode == RUN_LED_MODE_OFF) {
            MID_RunLED_Write(false);
        }
    }
}

/**
 * @brief 获取当前指示灯工作模式
 */
run_led_mode_t MID_RunLED_GetMode(void)
{
    return s_current_mode;
}

/**
 * @brief 指示灯周期步进更新 (在 50ms 任务中调用)
 * @param interval_ms 调用周期毫秒数 (如 50ms)
 */
void MID_RunLED_Process(uint16_t interval_ms)
{
    switch (s_current_mode) {
        case RUN_LED_MODE_STEADY_ON:
            if (!s_led_state) {
                MID_RunLED_Write(true);
            }
            break;

        case RUN_LED_MODE_OFF:
            if (s_led_state) {
                MID_RunLED_Write(false);
            }
            break;

        case RUN_LED_MODE_BLINK_RUN:
            // 运行慢闪: 250ms 翻转一次 (周期 500ms, 1Hz)
            s_timer_ms += interval_ms;
            if (s_timer_ms >= 250) {
                s_timer_ms = 0;
                MID_RunLED_Toggle();
            }
            break;

        case RUN_LED_MODE_BLINK_FAULT:
            // 故障急闪: 100ms 翻转一次 (周期 200ms, 5Hz)
            s_timer_ms += interval_ms;
            if (s_timer_ms >= 100) {
                s_timer_ms = 0;
                MID_RunLED_Toggle();
            }
            break;

        default:
            break;
    }
}
