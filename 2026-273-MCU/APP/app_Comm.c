#include "app_Comm.h"
#include "mid_modbus.h"
#include "app_pid.h"
#include "app_Data.h"

// 声明 FreeRTOS 队列句柄
QueueHandle_t g_motor_ctrl_queue = NULL;

// ========================== Modbus 回调函数 ==========================

static void Motor_Cmd_Callback_Generic(uint8_t motor_idx, uint8_t success)
{
    if (success) {
        g_sys_context.g_motor_status[motor_idx].current_cmd = g_sys_context.g_motor_status[motor_idx].target_cmd;
        g_sys_context.g_motor_status[motor_idx].retry_cnt   = 0;
    } else {
        g_sys_context.g_motor_status[motor_idx].retry_cnt++;
    }
}

static void Motor_Speed_Callback_Generic(uint8_t motor_idx, uint8_t success)
{
    if (success) {
        g_sys_context.g_motor_status[motor_idx].current_speed = g_sys_context.g_motor_status[motor_idx].target_speed;
        g_sys_context.g_motor_status[motor_idx].retry_cnt     = 0;
    } else {
        g_sys_context.g_motor_status[motor_idx].retry_cnt++;
    }
}

static void Motor_ReadHall_Callback_Generic(uint8_t motor_idx, uint16_t *pData, uint8_t success)
{
    if (success && pData != NULL) {
        // 抓取驱动器返回的原始 32 位相对位置值，回写至内存缓存层
        int32_t drive_relative_hall                        = (int32_t)(((uint32_t)pData[0] << 16) | pData[1]);
        g_sys_context.g_motor_status[motor_idx].hall_value = (uint32_t)drive_relative_hall;
        g_sys_context.g_motor_status[motor_idx].comm_error = 0;
    } else {
        g_sys_context.g_motor_status[motor_idx].comm_error = 1;
    }
}

#define DEFINE_MOTOR_CALLBACKS(num, idx)                                         \
    static void Motor##num##_Cmd_Callback(uint8_t success)                       \
    {                                                                            \
        Motor_Cmd_Callback_Generic(idx, success);                                \
    }                                                                            \
    static void Motor##num##_Speed_Callback(uint8_t success)                     \
    {                                                                            \
        Motor_Speed_Callback_Generic(idx, success);                              \
    }                                                                            \
    static void Motor##num##_ReadHall_Callback(uint16_t *pData, uint8_t success) \
    {                                                                            \
        Motor_ReadHall_Callback_Generic(idx, pData, success);                    \
    }

DEFINE_MOTOR_CALLBACKS(1, 0)
DEFINE_MOTOR_CALLBACKS(2, 1)
DEFINE_MOTOR_CALLBACKS(3, 2)
DEFINE_MOTOR_CALLBACKS(4, 3)

static const modbus_write_callback_t Motor_Cmd_Callbacks[4] = {
    Motor1_Cmd_Callback, Motor2_Cmd_Callback, Motor3_Cmd_Callback, Motor4_Cmd_Callback};

static const modbus_write_callback_t Motor_Speed_Callbacks[4] = {
    Motor1_Speed_Callback, Motor2_Speed_Callback, Motor3_Speed_Callback, Motor4_Speed_Callback};

static const modbus_read_callback_t Motor_ReadHall_Callbacks[4] = {
    Motor1_ReadHall_Callback, Motor2_ReadHall_Callback, Motor3_ReadHall_Callback, Motor4_ReadHall_Callback};

// ========================== Modbus 安全发送封装 ==========================

static bool App_Modbus_WriteSingleReg_Safe(Modbus_Master_t *m, uint16_t reg, uint16_t val, modbus_write_callback_t cb, uint8_t motor_idx)
{
    uint8_t retry = 0;
    while (m->state != MODBUS_STATE_IDLE && retry < 30) {
        MID_Modbus_Process_1ms();
        vTaskDelay(pdMS_TO_TICKS(1));
        retry++;
    }
    return MID_Modbus_WriteSingleReg(m, reg, val, cb);
}

static bool App_Modbus_ReadRegs_Safe(Modbus_Master_t *m, uint16_t reg, uint16_t count, modbus_read_callback_t cb, uint8_t motor_idx)
{
    uint8_t retry = 0;
    while (m->state != MODBUS_STATE_IDLE && retry < 30) {
        MID_Modbus_Process_1ms();
        vTaskDelay(pdMS_TO_TICKS(1));
        retry++;
    }
    return MID_Modbus_ReadRegs(m, reg, count, cb);
}

// ========================== 4路驱动器托管上电初始化序列 ==========================

static void App_Comm_InitHardwareSequence(void)
{
    Debug_Printf("[SYS] Starting 4-Axis Driver Hardware Initialization...\r\n");

    struct {
        uint16_t reg;
        uint16_t val;
        const char *name;
    } init_steps[] = {
        {0x200E, 0x0000, "Write Enable"},
        {0x2006, 0x0002, "Run Mode"},
        {0x2007, 0x0003, "Speed Mode"},
        {0x2001, 0x0000, "Set Speed 0"},
        {0x2000, 0x0005, "Start Drive"}};

    int num_steps = sizeof(init_steps) / sizeof(init_steps[0]);

    for (int step = 0; step < num_steps; step++) {
        Debug_Printf("[SYS] Init Step %d/%d (%s)... ", step + 1, num_steps, init_steps[step].name);

        for (int i = 0; i < 4; i++) {
            Modbus_Master_t *m = &modbus_masters[i];
            App_Modbus_WriteSingleReg_Safe(m, init_steps[step].reg, init_steps[step].val, Motor_Cmd_Callbacks[i], i);
        }

        uint8_t wait_ms = 0;
        bool step_ok    = false;
        while (wait_ms < 100) {
            MID_Modbus_Process_1ms();
            vTaskDelay(pdMS_TO_TICKS(1));
            wait_ms++;

            bool all_idle = true;
            for (int i = 0; i < 4; i++) {
                if (modbus_masters[i].state != MODBUS_STATE_IDLE) {
                    all_idle = false;
                    break;
                }
            }
            if (all_idle) {
                step_ok = true;
                break;
            }
        }
        Debug_Printf(step_ok ? "OK\r\n" : "Done\r\n");
    }

    g_sys_context.is_hardware_ready = true;
    Debug_Printf("[SYS] Hardware Initialization Finished! State -> READY\r\n");
}

// ========================== 485 并行 Modbus 轮询与调度任务 ==========================

void APP_CommTask(void *pvParameters)
{
    Motor_Ctrl_Msg_t ctrl_msg;

    g_motor_ctrl_queue = xQueueCreate(20, sizeof(Motor_Ctrl_Msg_t));

    Debug_Printf("[SYS] APP_CommTask Started.\r\n");
    Debug_Printf("[SYS] Loaded Flash Abs Halls: H0=%d, H1=%d, H2=%d, H3=%d | MaxTravel=%d\r\n",
                 app_data.motor_abs_halls[0],
                 app_data.motor_abs_halls[1],
                 app_data.motor_abs_halls[2],
                 app_data.motor_abs_halls[3],
                 app_data.max_travel_range);

    // 1. 执行托管的 4 路电机驱动器硬件初始化
    App_Comm_InitHardwareSequence();

    TickType_t xLastPollTick = xTaskGetTickCount();

    while (1) {
        // 2. 消费控制队列命令
        if (xQueueReceive(g_motor_ctrl_queue, &ctrl_msg, 0) == pdTRUE) {
            for (int i = 0; i < 4; i++) {
                if (ctrl_msg.motor_mask & (1 << i)) {
                    Modbus_Master_t *m = &modbus_masters[i];

                    switch (ctrl_msg.cmd_type) {
                        case CMD_INIT_SET_SPEED:
                            App_Modbus_WriteSingleReg_Safe(m, 0x2001, ctrl_msg.speed_rpm, Motor_Speed_Callbacks[i], i);
                            break;

                        case CMD_FORWARD:
                            App_Modbus_WriteSingleReg_Safe(m, 0x2000, 0x0001, Motor_Cmd_Callbacks[i], i);
                            break;

                        case CMD_REVERSE:
                            App_Modbus_WriteSingleReg_Safe(m, 0x2000, 0x0002, Motor_Cmd_Callbacks[i], i);
                            break;

                        case CMD_STOP:
                            App_Modbus_WriteSingleReg_Safe(m, 0x2000, 0x0009, Motor_Cmd_Callbacks[i], i);
                            break;

                        case CMD_READ_HALL:
                            App_Modbus_ReadRegs_Safe(m, 0x3013, 2, Motor_ReadHall_Callbacks[i], i);
                            break;

                        default:
                            break;
                    }
                }
            }
        }

        // 3. 定频（10ms 周期）自动向 4 路 Modbus 轮询读取当前最新霍尔高度
        if (xTaskGetTickCount() - xLastPollTick >= pdMS_TO_TICKS(10)) {
            xLastPollTick = xTaskGetTickCount();
            for (int i = 0; i < 4; i++) {
                Modbus_Master_t *m = &modbus_masters[i];
                if (m->state == MODBUS_STATE_IDLE) {
                    MID_Modbus_ReadRegs(m, 0x3013, 2, Motor_ReadHall_Callbacks[i]);
                }
            }
        }

        // 4. 推进 Modbus 状态机
        MID_Modbus_Process_1ms();

        // 5. 挂起 1ms 定时，释放 CPU
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
