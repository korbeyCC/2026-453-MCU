#include "app_Ctllogic.h"
#include "mid_signal.h"
#include "mid_led.h"

/**
 * @brief 核心业务控制逻辑任务
 * @note 负责接收外接信号和无线遥控事件，处理灯带与升降状态逻辑
 */
void APP_CtllogicTask(void *pvParameters)
{
    MID_SIGNAL_Msg sig_msg;

    // 初始化中间层灯带控制
    MID_LED_Init();

    while (1)
    {
        // 阻塞读取外接信号与无线遥控事件队列
        if (MID_Signal_GetEvent(&sig_msg, portMAX_DELAY) == pdTRUE)
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
                        // 遥控下键按下，关闭所有灯带 (或执行其它逻辑)
                        MID_LED_Write(MID_LED_1, false);
                        MID_LED_Write(MID_LED_2, false);
                        break;
                        
                    case MID_SIGNAL_REMOT_4:
                        // 遥控上键按下，开启所有灯带 (或执行其它逻辑)
                        MID_LED_Write(MID_LED_1, true);
                        MID_LED_Write(MID_LED_2, true);
                        break;
                        
                    case MID_SIGNAL_REMOT_5:
                        // 遥控器 5 号键按下
                        break;
                        
                    case MID_SIGNAL_BUTON_DW:
                        // 外接下按键按下
                        break;
                        
                    case MID_SIGNAL_BUTON_UP:
                        // 外接上按键按下
                        break;
                        
                    default:
                        break;
                }
            }
        }
    }
}
