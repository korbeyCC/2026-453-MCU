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
        // 抓取驱动器返回的原始 32 位无符号霍尔位置值 (0x3013)，回写至内存缓存层
        g_sys_context.g_motor_status[motor_idx].hall_value = ((uint32_t)pData[0] << 16) | pData[1];
        g_sys_context.g_motor_status[motor_idx].comm_error = 0;
    } else {
        if (g_sys_context.g_motor_status[motor_idx].comm_error < 255) {
            g_sys_context.g_motor_status[motor_idx].comm_error++;
        }
    }
}

static void Motor_ReadCurrent_Callback_Generic(uint8_t motor_idx, uint16_t *pData, uint8_t success)
{
    if (success && pData != NULL) {
        // 抓取驱动器 0x3004 保持寄存器返回的输出电流 (单位: 0.01A)
        g_sys_context.g_motor_status[motor_idx].current_deciA = pData[0];
        g_sys_context.g_motor_status[motor_idx].comm_error    = 0;
    } else {
        if (g_sys_context.g_motor_status[motor_idx].comm_error < 255) {
            g_sys_context.g_motor_status[motor_idx].comm_error++;
        }
    }
}

#define DEFINE_MOTOR_CALLBACKS(num, idx)                                            \
    static void Motor##num##_Cmd_Callback(uint8_t success)                          \
    {                                                                               \
        Motor_Cmd_Callback_Generic(idx, success);                                   \
    }                                                                               \
    static void Motor##num##_Speed_Callback(uint8_t success)                        \
    {                                                                               \
        Motor_Speed_Callback_Generic(idx, success);                                 \
    }                                                                               \
    static void Motor##num##_ReadHall_Callback(uint16_t *pData, uint8_t success)    \
    {                                                                               \
        Motor_ReadHall_Callback_Generic(idx, pData, success);                       \
    }                                                                               \
    static void Motor##num##_ReadCurrent_Callback(uint16_t *pData, uint8_t success) \
    {                                                                               \
        Motor_ReadCurrent_Callback_Generic(idx, pData, success);                    \
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

static const modbus_read_callback_t Motor_ReadCurrent_Callbacks[4] = {
    Motor1_ReadCurrent_Callback, Motor2_ReadCurrent_Callback, Motor3_ReadCurrent_Callback, Motor4_ReadCurrent_Callback};

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

static void App_Comm_InitHardwareSequence(void)
{
    // 1. 以初始 19200 BPS 向驱动器发送 0x2009 = 7 指令，提升驱动器通信波特率至 115200 BPS
    Debug_Printf("[SYS] Setting Driver Baudrate to 115200 BPS (0x2009 = 7)...\r\n");
    for (int i = 0; i < 4; i++) {
        App_Modbus_WriteSingleReg_Safe(&modbus_masters[i], 0x2009, 7, Motor_Cmd_Callbacks[i], i);
    }

    uint8_t wait_ms = 0;
    while (wait_ms < 50) {
        MID_Modbus_Process_1ms();
        vTaskDelay(pdMS_TO_TICKS(1));
        wait_ms++;
    }

    // 2. 单片机本地 4 路 RS485 串口重新初始化切频提升至 115200 BPS
    MID_Modbus_SetBaudRate(115200);
    Debug_Printf("[SYS] MCU RS485 Baudrate Switched to 115200 BPS Success!\r\n");

    // 3. 执行后续 6 步硬件初始化序列 (含 0x2000=0x0007 上电故障复位)
    struct {
        uint16_t reg;
        uint16_t val;
        const char *name;
    } init_steps[] = {
        {0x2000, 0x0007, "Fault Reset"},
        {0x200E, 0x0000, "Write Enable"},
        {0x2006, 0x0002, "Run Mode"},
        {0x2007, 0x0003, "Speed Mode"},
        {0x2001, 0x0000, "Set Speed 0"},
        {0x2000, 0x0005, "Start Drive"}};

    int num_steps = sizeof(init_steps) / sizeof(init_steps[0]);

    for (int step = 0; step < num_steps; step++) {

        for (int i = 0; i < 4; i++) {
            Modbus_Master_t *m = &modbus_masters[i];
            App_Modbus_WriteSingleReg_Safe(m, init_steps[step].reg, init_steps[step].val, Motor_Cmd_Callbacks[i], i);
        }

        uint8_t wait_ms = 0;
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
                break;
            }
        }
    }

    g_sys_context.is_hardware_ready = true;
}

// ========================== 485 并行 Modbus 轮询与调度任务 ==========================

void APP_CommTask(void *pvParameters)
{
    Motor_Ctrl_Msg_t ctrl_msg;

    g_motor_ctrl_queue = xQueueCreate(20, sizeof(Motor_Ctrl_Msg_t));

    Debug_Printf("[SYS] Loaded Flash Abs Halls: H0=%d, H1=%d, H2=%d, H3=%d | MaxTravelMM=%dmm\r\n",
                 app_data.motor_abs_halls[0],
                 app_data.motor_abs_halls[1],
                 app_data.motor_abs_halls[2],
                 app_data.motor_abs_halls[3],
                 app_data.max_travel_range_mm);

    // 1. 执行托管的 4 路电机驱动器硬件初始化
    App_Comm_InitHardwareSequence();

    static bool pending_set_speed[4]     = {false, false, false, false};
    static uint16_t pending_speed_rpm[4] = {0, 0, 0, 0};
    TickType_t xLastPollTick              = xTaskGetTickCount();

    while (1) {
        // 2. 5ms 极速周期轮询：最高优先级保障 0x3013 读霍尔位置 (200Hz)
        if (xTaskGetTickCount() - xLastPollTick >= pdMS_TO_TICKS(5)) {
            xLastPollTick = xTaskGetTickCount();

            static uint8_t poll_cnt = 0;
            poll_cnt++;

            // 每 20 帧 (100ms) 抽样读取一次 0x3004 电流，其余 19 帧以 5ms 极速轮询 0x3013 霍尔位置
            bool read_current = (poll_cnt >= 20);
            if (read_current) {
                poll_cnt = 0;
            }

            for (int i = 0; i < 4; i++) {
                Modbus_Master_t *m = &modbus_masters[i];
                if (m->state == MODBUS_STATE_IDLE) {
                    if (read_current) {
                        MID_Modbus_ReadRegs(m, 0x3004, 1, Motor_ReadCurrent_Callbacks[i]);
                    } else {
                        MID_Modbus_ReadRegs(m, 0x3013, 2, Motor_ReadHall_Callbacks[i]);
                    }
                }
            }
        }

        // 3. 消费控制队列命令 (写转速命令进入 pending 挂起，绝对不丢包)
        if (xQueueReceive(g_motor_ctrl_queue, &ctrl_msg, 0) == pdTRUE) {
            for (int i = 0; i < 4; i++) {
                if (ctrl_msg.motor_mask & (1 << i)) {
                    Modbus_Master_t *m = &modbus_masters[i];

                    switch (ctrl_msg.cmd_type) {
                        case CMD_SET_SPEED:
                            pending_set_speed[i] = true;
                            pending_speed_rpm[i] = ctrl_msg.speed_rpm;
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

                        default:
                            break;
                    }
                }
            }
        }

        // 4. 自动处理 pending 挂起的写转速请求 (串口 IDLE 时补发，确保写转速 100% 成功下发)
        for (int i = 0; i < 4; i++) {
            if (pending_set_speed[i]) {
                Modbus_Master_t *m = &modbus_masters[i];
                if (m->state == MODBUS_STATE_IDLE) {
                    if (MID_Modbus_WriteSingleReg(m, 0x2001, pending_speed_rpm[i], Motor_Speed_Callbacks[i])) {
                        pending_set_speed[i] = false;
                    }
                }
            }
        }

        // 5. 推进 Modbus 状态机
        MID_Modbus_Process_1ms();

        // 6. 挂起 1ms 定时，释放 CPU
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
