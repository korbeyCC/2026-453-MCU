#include "app_control.h"
#include "mid_signal.h"
#include "mid_led.h"
#include "mid_modbus.h"
#include "app_Comm.h"

/**
 * @brief 核心业务控制任务
 * @note  负责监控按键和遥控器信号，运算电机同步控制量，并通过队列向通信任务下发控制指令
 */
void APP_ControlTask(void *pvParameters)
{
    MID_SIGNAL_Msg sig_msg;

    // 静态状态变量，用于记录应用层当前的运行控制动作
    static uint8_t s_ctrl_state = CMD_STOP;

    // 初始化中间层灯带控制
    MID_LED_Init();

    while (1) {
        // 设为 50ms 超时等待，保证按键和事件的响应流畅度，又避免空等轮询消耗 CPU
        if (MID_Signal_GetEvent(&sig_msg, pdMS_TO_TICKS(50)) == pdTRUE) {

            // ==================== 仅处理按键按下触发事件 (TRIGGER) ====================
            if (sig_msg.event == MID_SIGNAL_EVT_TRIGGER) {

                // 【规则 1】：如果当前处于运行状态 (上升或下降)，任何按键按下都立即触发停机
                if (s_ctrl_state != CMD_STOP) {
                    Motor_Ctrl_Msg_t ctrl_msg;
                    ctrl_msg.cmd_type  = CMD_STOP;
                    ctrl_msg.speed_rpm = 0;
                    xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);

                    s_ctrl_state = CMD_STOP;

                    // 【消费并清空事件】：成功下发停机指令后，立即非阻塞读空/消费掉队列中由于松手或抖动产生的所有残留事件
                    MID_SIGNAL_Msg dummy_msg;
                    while (MID_Signal_GetEvent(&dummy_msg, 0) == pdTRUE);
                }
                // 【规则 2】：如果当前处于静止状态，按下对应按键执行相应动作 (起步)
                else {
                    Motor_Ctrl_Msg_t ctrl_msg;
                    bool action_valid = false;

                    switch (sig_msg.signal_id) {
                        case MID_SIGNAL_REMOT_1:
                            // 遥控器 1 号键按下，切换灯带 1 状态
                            MID_LED_Toggle(MID_LED_1);
                            break;

                        case MID_SIGNAL_REMOT_2:
                            // 遥控器 2 号键按下，切换灯带 2 状态
                            MID_LED_Toggle(MID_LED_2);
                            break;

                        case MID_SIGNAL_REMOT_3:
                            // 遥控下键按下，关闭所有灯带，并启动电机下行 (反转)
                            MID_LED_Write(MID_LED_1, false);
                            MID_LED_Write(MID_LED_2, false);

                            ctrl_msg.cmd_type  = CMD_REVERSE;
                            ctrl_msg.speed_rpm = 100;
                            xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);
                            s_ctrl_state = CMD_REVERSE;
                            action_valid = true;
                            break;

                        case MID_SIGNAL_REMOT_4:
                            // 遥控上键按下，开启所有灯带，并启动电机上行 (正转)
                            MID_LED_Write(MID_LED_1, true);
                            MID_LED_Write(MID_LED_2, true);

                            ctrl_msg.cmd_type  = CMD_FORWARD;
                            ctrl_msg.speed_rpm = 100;
                            xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);
                            s_ctrl_state = CMD_FORWARD;
                            action_valid = true;
                            break;

                        case MID_SIGNAL_BUTON_DW:
                            // 物理下行按键按下，启动电机下行 (反转)
                            ctrl_msg.cmd_type  = CMD_REVERSE;
                            ctrl_msg.speed_rpm = 100;
                            xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);
                            s_ctrl_state = CMD_REVERSE;
                            action_valid = true;
                            break;

                        case MID_SIGNAL_BUTON_UP:
                            // 物理上行按键按下，启动电机上行 (正转)
                            ctrl_msg.cmd_type  = CMD_FORWARD;
                            ctrl_msg.speed_rpm = 100;
                            xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);
                            s_ctrl_state = CMD_FORWARD;
                            action_valid = true;
                            break;

                        default:
                            break;
                    }

                    if (action_valid) {
                        // 【消费并清空事件】：成功下发启动指令后，立即非阻塞读空/消费掉后续的释放及抖动残留事件
                        MID_SIGNAL_Msg dummy_msg;
                        while (MID_Signal_GetEvent(&dummy_msg, 0) == pdTRUE);
                    }
                }
            }
        }
    }
}
