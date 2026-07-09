#include "app_Ctllogic.h"
#include "mid_signal.h"
#include "mid_led.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

/**
 * @brief 核心业务控制逻辑任务
 * @note 负责接收外接信号和无线遥控事件，处理灯带与升降状态逻辑，并发送 485 链路测试包
 */
void APP_CtllogicTask(void *pvParameters)
{
    MID_SIGNAL_Msg sig_msg;
    
    // 准备 485 测试数据
    uint8_t tx_data1[] = "USART1 485 Test\r\n";
    uint8_t tx_data2[] = "USART2 485 Test\r\n";
    uint8_t tx_data3[] = "USART3 485 Test\r\n";
    uint8_t tx_data4[] = "UART4 485 Test\r\n";
    
    uint32_t last_tx_tick = 0;

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
        
        // 485 链路硬件循环测试：每 1000ms 自动向 4 路 485 发送一次测试报文
        uint32_t current_tick = xTaskGetTickCount();
        if (current_tick - last_tx_tick >= pdMS_TO_TICKS(1000))
        {
            last_tx_tick = current_tick;
            
            HAL_UART_Transmit(&huart1, tx_data1, sizeof(tx_data1) - 1, 100);
            HAL_UART_Transmit(&huart2, tx_data2, sizeof(tx_data2) - 1, 100);
            HAL_UART_Transmit(&huart3, tx_data3, sizeof(tx_data3) - 1, 100);
            HAL_UART_Transmit(&huart4, tx_data4, sizeof(tx_data4) - 1, 100);
        }
    }
}
