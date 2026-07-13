#include "app_control.h"
#include "mid_signal.h"
#include "mid_led.h"
#include "mid_modbus.h"
#include "app_Comm.h"

// 4路电机的PID控制器结构体实例，用于升降同步的高度修偏
static APP_PID_Handle_t motor_pids[4];

/**
 * @brief 核心业务控制任务
 * @note  负责监控按键和遥控器信号，运算电机同步控制量，并通过队列向通信任务下发控制指令
 */
void APP_ControlTask(void *pvParameters)
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
            // ==================== 1. 按键/遥控器 按下触发 (TRIGGER) ====================
            if (sig_msg.event == MID_SIGNAL_EVT_TRIGGER)
            {
                Motor_Ctrl_Msg_t ctrl_msg;
                
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
                        // 遥控下键按下，关闭所有灯带，并启动电机下行 (反转，速度 150 RPM)
                        MID_LED_Write(MID_LED_1, false);
                        MID_LED_Write(MID_LED_2, false);
                        
                        ctrl_msg.cmd_type = CMD_REVERSE;
                        ctrl_msg.speed_rpm = 150;
                        xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);
                        break;
                        
                    case MID_SIGNAL_REMOT_4:
                        // 遥控上键按下，开启所有灯带，并启动电机上行 (正转，速度 150 RPM)
                        MID_LED_Write(MID_LED_1, true);
                        MID_LED_Write(MID_LED_2, true);
                        
                        ctrl_msg.cmd_type = CMD_FORWARD;
                        ctrl_msg.speed_rpm = 150;
                        xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);
                        break;
                        
                    case MID_SIGNAL_BUTON_DW:
                        // 物理下行按键按下，启动电机下行 (反转，速度 150 RPM)
                        ctrl_msg.cmd_type = CMD_REVERSE;
                        ctrl_msg.speed_rpm = 150;
                        xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);
                        break;
                        
                    case MID_SIGNAL_BUTON_UP:
                        // 物理上行按键按下，启动电机上行 (正转，速度 150 RPM)
                        ctrl_msg.cmd_type = CMD_FORWARD;
                        ctrl_msg.speed_rpm = 150;
                        xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);
                        break;
                        
                    default:
                        break;
                }
            }
            // ==================== 2. 按键/遥控器 松开触发 (RELEASE) ====================
            else if (sig_msg.event == MID_SIGNAL_EVT_RELEASE)
            {
                // 如果是遥控上下键、或者物理上下按键被松开，则发送停机命令 (点动停止)
                if (sig_msg.signal_id == MID_SIGNAL_REMOT_4 || 
                    sig_msg.signal_id == MID_SIGNAL_REMOT_3 ||
                    sig_msg.signal_id == MID_SIGNAL_BUTON_UP ||
                    sig_msg.signal_id == MID_SIGNAL_BUTON_DW)
                {
                    Motor_Ctrl_Msg_t ctrl_msg;
                    ctrl_msg.cmd_type = CMD_STOP;
                    ctrl_msg.speed_rpm = 0;
                    xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);
                }
            }
        }
    }
}
