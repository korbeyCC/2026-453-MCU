#include "mid_buzzer.h"
#include "main.h"

static buzzer_mode_t s_buzzer_mode = BUZZER_MODE_MUTE;
static uint16_t s_pattern_timer_ms = 0;
static uint16_t s_oneshot_timer_ms = 0;
static bool s_buzzer_state         = false; // 当前输出电平状态

/**
 * @brief 蜂鸣器初始化 (PD2, 默认静音)
 */
void MID_Buzzer_Init(void)
{
    // 防御性硬件初始化：确保 PD2 强制配置为推挽输出模式
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = BEEP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStruct);

    s_buzzer_mode      = BUZZER_MODE_MUTE;
    s_pattern_timer_ms = 0;
    s_oneshot_timer_ms = 0;
    s_buzzer_state     = false;
    MID_Buzzer_Write(false);
}

/**
 * @brief 控制蜂鸣器输出
 * @param on true: 鸣叫 (PD2 输出高电平驱动 NPN 三极管饱和导通), false: 静音 (PD2 输出低电平)
 */
void MID_Buzzer_Write(bool on)
{
    s_buzzer_state = on;
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 翻转蜂鸣器输出电平
 */
void MID_Buzzer_Toggle(void)
{
    s_buzzer_state = !s_buzzer_state;
    HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin);
}

/**
 * @brief 设置蜂鸣器背景工作模式
 */
void MID_Buzzer_SetMode(buzzer_mode_t mode)
{
    if (s_buzzer_mode != mode) {
        s_buzzer_mode      = mode;
        s_pattern_timer_ms = 0;
        if (mode == BUZZER_MODE_MUTE && s_oneshot_timer_ms == 0) {
            MID_Buzzer_Write(false);
        }
    }
}

/**
 * @brief 获取当前蜂鸣器工作模式
 */
buzzer_mode_t MID_Buzzer_GetMode(void)
{
    return s_buzzer_mode;
}

/**
 * @brief 触发一次性单次蜂鸣 (如按键提示音、完成提示音)
 * @param duration_ms 蜂鸣持续时长 (ms)
 */
void MID_Buzzer_TriggerBeep(uint16_t duration_ms)
{
    s_oneshot_timer_ms = duration_ms;
    MID_Buzzer_Write(true);
}

#define BEEP_TEST_TOGGLE_1S 0 // 1: 启用每秒翻转测试模式, 0: 恢复正常模式

/**
 * @brief 蜂鸣器周期步进更新 (在 50ms 任务中调用)
 * @param interval_ms 调用周期毫秒数 (如 50ms)
 */
void MID_Buzzer_Process(uint16_t interval_ms)
{
#if BEEP_TEST_TOGGLE_1S
    // 【临时硬件测试】每秒翻转一次蜂鸣器状态 (1000ms 鸣叫 / 1000ms 静音)
    static uint16_t s_test_toggle_timer = 0;
    (void)s_pattern_timer_ms;
    s_test_toggle_timer += interval_ms;
    if (s_test_toggle_timer >= 2000) {
        s_test_toggle_timer = 0;
        MID_Buzzer_Toggle();
    }
#else
    // 优先处理一次性触发的按键或事件短蜂鸣
    if (s_oneshot_timer_ms > 0) {
        if (s_oneshot_timer_ms > interval_ms) {
            s_oneshot_timer_ms -= interval_ms;
        } else {
            s_oneshot_timer_ms = 0;
            // 一次性蜂鸣结束，若背景模式为静音则关断
            if (s_buzzer_mode == BUZZER_MODE_MUTE) {
                MID_Buzzer_Write(false);
            }
        }
        return;
    }

    // 处理背景模式节拍
    switch (s_buzzer_mode) {
        case BUZZER_MODE_MUTE:
            if (s_buzzer_state) {
                MID_Buzzer_Write(false);
            }
            break;

        case BUZZER_MODE_RUNNING:
            // 运行提示鸣叫: 每 1000ms 蜂鸣 100ms
            s_pattern_timer_ms += interval_ms;
            if (s_pattern_timer_ms < 100) {
                if (!s_buzzer_state) MID_Buzzer_Write(true);
            } else if (s_pattern_timer_ms < 1000) {
                if (s_buzzer_state) MID_Buzzer_Write(false);
            } else {
                s_pattern_timer_ms = 0;
                MID_Buzzer_Write(true);
            }
            break;

        case BUZZER_MODE_ALARM:
            // 故障急促报警: 100ms 鸣 / 100ms 停 (周期 200ms)
            s_pattern_timer_ms += interval_ms;
            if (s_pattern_timer_ms < 100) {
                if (!s_buzzer_state) MID_Buzzer_Write(true);
            } else if (s_pattern_timer_ms < 200) {
                if (s_buzzer_state) MID_Buzzer_Write(false);
            } else {
                s_pattern_timer_ms = 0;
                MID_Buzzer_Write(true);
            }
            break;

        default:
            break;
    }
#endif
}
