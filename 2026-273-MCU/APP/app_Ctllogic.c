#include "app_Ctllogic.h"
#include "mid_signal.h"
#include "mid_led.h"
#include "mid_modbus.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

// 4路电机的PID控制器结构体实例，用于升降同步的高度修偏
static APP_PID_Handle_t motor_pids[4];

/**
 * @brief 核心业务控制逻辑任务
 * @note 负责接收外接信号和无线遥控事件，处理灯带与升降状态逻辑，并周期发起 Modbus 链路测试
 */
void APP_CtllogicTask(void *pvParameters)
{
    MID_SIGNAL_Msg sig_msg;

    // 初始化 4 路立柱电机的同步 PI 控制器 (使用导入的 app_pid.c 算法)
    // 暂定默认参数：Kp = 2.5f, Ki = 0.1f, Kd = 0.0f (不启用微分项)
    // 高度差目标 Target = 0.0f, 速度微调限幅范围 [-150.0f, 150.0f], 积分饱和限幅 20.0f
    for (int i = 0; i < 4; i++)
    {
        APP_PID_Init(&motor_pids[i], 2.5f, 0.1f, 0.0f, 0.0f, 150.0f, -150.0f, 20.0f);
    }

    // 初始化中间层灯带控制
    MID_LED_Init();

    while (1)
    {
        // 设为 50ms 超时等待，保证按键和事件的响应流畅度，又避免空等轮询消耗 CPU
        if (MID_Signal_GetEvent(&sig_msg, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            // 仅在信号触发生效(TRIGGER)时执行对应的控制动作
            if (sig_msg.event == MID_SIGNAL_EVT_TRIGGER)
            {
                switch (sig_msg.signal_id)
                {
                    case MID_SIGNAL_REMOT_1:
                        // 遥控器 1 号键按下，切换灯带 1 状态
                        MID_LED_Toggle(MID_LED_1);
                        break;
                        
                    case MID_SIGNAL_REMOT_2:
                        // 遥控器 2 号键按下，切换灯带 2 状态
                        MID_LED_Toggle(MID_LED_2);
                        break;
                        
                    case MID_SIGNAL_REMOT_3:
                        // 遥控下键按下，关闭所有灯带
                        MID_LED_Write(MID_LED_1, false);
                        MID_LED_Write(MID_LED_2, false);
                        break;
                        
                    case MID_SIGNAL_REMOT_4:
                        // 遥控上键按下，开启所有灯带
                        MID_LED_Write(MID_LED_1, true);
                        MID_LED_Write(MID_LED_2, true);
                        break;
                        
                    case MID_SIGNAL_REMOT_5:
                        break;
                        
                    case MID_SIGNAL_BUTON_DW:
                        break;
                        
                    case MID_SIGNAL_BUTON_UP:
                        break;
                        
                    default:
                        break;
                }
            }
        }
        
    }
}
