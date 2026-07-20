#include "app_Comm.h"
#include "mid_modbus.h"
#include "app_pid.h"
#include "app_Data.h"

// 声明 FreeRTOS 队列句柄
QueueHandle_t g_motor_ctrl_queue = NULL;

// ========================== Modbus 通用回调泛型函数 ==========================

/**
 * @brief  通用动作指令写入回调
 */
static void Motor_Cmd_Callback_Generic(uint8_t motor_idx, uint8_t success)
{
    if (success) {
        g_sys_context.g_motor_status[motor_idx].current_cmd = g_sys_context.g_motor_status[motor_idx].target_cmd;
        g_sys_context.g_motor_status[motor_idx].retry_cnt = 0;
        
        // 触发 CMD 事件位
        xEventGroupSetBits(g_sys_context.event_group, CMD_EVENT_BIT(motor_idx));
    } else {
        g_sys_context.g_motor_status[motor_idx].retry_cnt++;
        
        // 配置/初始化阶段连续 3 次写指令超时，则强制设置事件答复，防止卡死
        if (g_sys_context.system_step >= SYS_STEP_INIT_WRITE_ENABLE && 
            g_sys_context.system_step <= SYS_STEP_INIT_START_RUN)
        {
            if (g_sys_context.g_motor_status[motor_idx].retry_cnt >= 3)
            {
                g_sys_context.g_motor_status[motor_idx].retry_cnt = 0;
                xEventGroupSetBits(g_sys_context.event_group, CMD_EVENT_BIT(motor_idx));
                Debug_Printf("[ERR] Motor %d cmd %d Write Timeout, skipped!\r\n", motor_idx, g_sys_context.g_motor_status[motor_idx].target_cmd);
            }
        }
    }
}

/**
 * @brief  通用目标速度写入回调
 */
static void Motor_Speed_Callback_Generic(uint8_t motor_idx, uint8_t success)
{
    if (success) {
        g_sys_context.g_motor_status[motor_idx].current_speed = g_sys_context.g_motor_status[motor_idx].target_speed;
        g_sys_context.g_motor_status[motor_idx].retry_cnt = 0;
        
        // 触发 SPEED 事件位
        xEventGroupSetBits(g_sys_context.event_group, SPEED_EVENT_BIT(motor_idx));
    } else {
        g_sys_context.g_motor_status[motor_idx].retry_cnt++;
        
        // 起跑等待期间，连续 3 次写速度超时，强制设置 SPEED 答复，防止起跑卡死
        if (g_sys_context.system_step == SYS_STEP_TOTAL_FORWARD || 
            g_sys_context.system_step == SYS_STEP_TOTAL_REVERSE)
        {
            if (g_sys_context.g_motor_status[motor_idx].retry_cnt >= 3)
            {
                g_sys_context.g_motor_status[motor_idx].retry_cnt = 0;
                xEventGroupSetBits(g_sys_context.event_group, SPEED_EVENT_BIT(motor_idx));
            }
        }
    }
}

/**
 * @brief  通用霍尔读取回调
 */
static void Motor_ReadHall_Callback_Generic(uint8_t motor_idx, uint16_t *pData, uint8_t success)
{
    if (success && pData != NULL) {
        // 1. 仅抓取驱动器返回的原始 32 位相对位置值，回写至监控数据层
        int32_t drive_relative_hall = (int32_t)(((uint32_t)pData[0] << 16) | pData[1]);
        g_sys_context.g_motor_status[motor_idx].hall_value = (uint32_t)drive_relative_hall;
        g_sys_context.g_motor_status[motor_idx].comm_error = 0;
    } else {
        if (g_sys_context.g_motor_status[motor_idx].comm_error == 0) {
            Debug_Printf("[ERR] Motor %d Comm Timeout!\r\n", motor_idx);
        }
        g_sys_context.g_motor_status[motor_idx].comm_error = 1;
    }
    
    // 触发霍尔读取完成事件位 (READ)
    xEventGroupSetBits(g_sys_context.event_group, READ_EVENT_BIT(motor_idx));
}

// ========================== 回调函数批量宏展开定义 ==========================

#define DEFINE_MOTOR_CALLBACKS(num, idx) \
    static void Motor##num##_Cmd_Callback(uint8_t success) { \
        Motor_Cmd_Callback_Generic(idx, success); \
    } \
    static void Motor##num##_Speed_Callback(uint8_t success) { \
        Motor_Speed_Callback_Generic(idx, success); \
    } \
    static void Motor##num##_ReadHall_Callback(uint16_t *pData, uint8_t success) { \
        Motor_ReadHall_Callback_Generic(idx, pData, success); \
    }

DEFINE_MOTOR_CALLBACKS(1, 0)
DEFINE_MOTOR_CALLBACKS(2, 1)
DEFINE_MOTOR_CALLBACKS(3, 2)
DEFINE_MOTOR_CALLBACKS(4, 3)

static const modbus_write_callback_t Motor_Cmd_Callbacks[4] = {
    Motor1_Cmd_Callback, Motor2_Cmd_Callback, Motor3_Cmd_Callback, Motor4_Cmd_Callback
};

static const modbus_write_callback_t Motor_Speed_Callbacks[4] = {
    Motor1_Speed_Callback, Motor2_Speed_Callback, Motor3_Speed_Callback, Motor4_Speed_Callback
};

static const modbus_read_callback_t Motor_ReadHall_Callbacks[4] = {
    Motor1_ReadHall_Callback, Motor2_ReadHall_Callback, Motor3_ReadHall_Callback, Motor4_ReadHall_Callback
};

/**
 * @brief  485 并行 Modbus 轮询与超时管理任务
 * @note   【被动化与 1ms 时基对齐】
 *         1. 彻底移除了对 target_cmd/target_speed 的改写动作。
 *         2. 采用非阻塞(0超时)读取队列配合严格 vTaskDelay(1) 调度，保证底层 MID_Modbus_Process_1ms() 拥有高精度物理轮询时基。
 */
void APP_CommTask(void *pvParameters)
{
    Motor_Ctrl_Msg_t ctrl_msg;
    
    g_motor_ctrl_queue = xQueueCreate(10, sizeof(Motor_Ctrl_Msg_t));
    
    Debug_Printf("[SYS] APP_CommTask Started, Waiting for Control Msg...\r\n");
    Debug_Printf("[SYS] Loaded Flash Abs Halls: H0=%d, H1=%d, H2=%d, H3=%d | MaxTravel=%d\r\n",
                 app_data.motor_abs_halls[0],
                 app_data.motor_abs_halls[1],
                 app_data.motor_abs_halls[2],
                 app_data.motor_abs_halls[3],
                 app_data.max_travel_range);
    
    while (1)
    {
        // 1. 0 延时非阻塞拉取队列，实现最高实时性的事件分发
        if (xQueueReceive(g_motor_ctrl_queue, &ctrl_msg, 0) == pdTRUE)
        {
            // 2. 根据消息指示的电机掩码与具体控制指令字，立刻执行并行 Modbus 发起，无脑直发
            for (int i = 0; i < 4; i++)
            {
                if (ctrl_msg.motor_mask & (1 << i))
                {
                    Modbus_Master_t *m = &modbus_masters[i];
                    
                    switch (ctrl_msg.cmd_type)
                    {
                        // 初始化与写配置阶段
                        case CMD_INIT_WRITE_ENABLE:
                            MID_Modbus_WriteSingleReg(m, 0x200E, 0x0000, Motor_Cmd_Callbacks[i]);
                            break;
                            
                        case CMD_INIT_RUN_MODE:
                            MID_Modbus_WriteSingleReg(m, 0x2006, 0x0002, Motor_Cmd_Callbacks[i]);
                            break;
                            
                        case CMD_INIT_SPEED_MODE:
                            MID_Modbus_WriteSingleReg(m, 0x2007, 0x0003, Motor_Cmd_Callbacks[i]);
                            break;
                            
                        case CMD_INIT_SET_SPEED:
                            MID_Modbus_WriteSingleReg(m, 0x2001, ctrl_msg.speed_rpm, Motor_Speed_Callbacks[i]);
                            break;
                            
                        case CMD_INIT_START_RUN:
                            MID_Modbus_WriteSingleReg(m, 0x2000, 0x0005, Motor_Cmd_Callbacks[i]);
                            break;
                            
                        // 运行控制阶段
                        case CMD_FORWARD:
                            MID_Modbus_WriteSingleReg(m, 0x2000, 0x0001, Motor_Cmd_Callbacks[i]);
                            break;
                            
                        case CMD_REVERSE:
                            MID_Modbus_WriteSingleReg(m, 0x2000, 0x0002, Motor_Cmd_Callbacks[i]);
                            break;
                            
                        case CMD_STOP:
                            MID_Modbus_WriteSingleReg(m, 0x2000, 0x0009, Motor_Cmd_Callbacks[i]);
                            break;
                            
                        // 读霍尔高度动作
                        case CMD_READ_HALL:
                            MID_Modbus_ReadRegs(m, 0x3013, 2, Motor_ReadHall_Callbacks[i]);
                            break;
                            
                        default:
                            break;
                    }
                }
            }
        }
        
        // 3. 推进底层 4 路串口 Modbus 主站的状态机，保持 1ms 物理调度精度
        MID_Modbus_Process_1ms();
        
        // 4. 挂起 1ms 定时，释放 CPU，保证 1ms 物理时基轮询的稳定精准
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
