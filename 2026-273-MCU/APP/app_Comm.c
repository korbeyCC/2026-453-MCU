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
        g_sys_context.hall_update_seq[motor_idx]++; // 霍尔成功更新打卡！
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

// ========================== 485 并行 Modbus 硬件初始化序列 ==========================

static void App_Comm_InitHardwareSequence(void)
{
    // 1. 以初始 19200 BPS 向驱动器发送 0x2009 = 7 指令，提升驱动器通信波特率至 115200 BPS
    for (int i = 0; i < 4; i++) {
        MID_Modbus_WriteSingleReg(&modbus_masters[i], 0x2009, 7, Motor_Cmd_Callbacks[i]);
    }

    uint8_t wait_ms = 0;
    while (wait_ms < 50) {
        MID_Modbus_Process_1ms();
        vTaskDelay(pdMS_TO_TICKS(1));
        wait_ms++;
    }

    // 2. 单片机本地 4 路 RS485 串口重新初始化切频提升至 115200 BPS
    MID_Modbus_SetBaudRate(115200);

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
        {0x2001, 300, "Set Speed 300"},
        {0x2000, 0x0005, "Start Drive"}};

    int num_steps = sizeof(init_steps) / sizeof(init_steps[0]);

    for (int step = 0; step < num_steps; step++) {
        for (int i = 0; i < 4; i++) {
            Modbus_Master_t *m = &modbus_masters[i];
            MID_Modbus_WriteSingleReg(m, init_steps[step].reg, init_steps[step].val, Motor_Cmd_Callbacks[i]);
        }

        uint8_t wait_cnt = 0;
        while (wait_cnt < 100) {
            MID_Modbus_Process_1ms();
            vTaskDelay(pdMS_TO_TICKS(1));
            wait_cnt++;

            bool all_idle = true;
            for (int i = 0; i < 4; i++) {
                if (modbus_masters[i].state != MODBUS_STATE_IDLE) {
                    all_idle = false;
                    break;
                }
            }
            if (all_idle) break;
        }
    }

    g_sys_context.is_hardware_ready = true;
}

// ========================== 485 并行 Modbus 轮询与调度任务 ==========================

void APP_CommTask(void *pvParameters)
{
    Motor_Ctrl_Msg_t ctrl_msg;

    g_motor_ctrl_queue = xQueueCreate(20, sizeof(Motor_Ctrl_Msg_t));

    Debug_Printf("[SYS] System Init: AbsHalls=[%d,%d,%d,%d], MinMountHalls=[%d,%d,%d,%d], MaxTravelRange=%dmm\r\n",
                 app_data.motor_abs_halls[0],
                 app_data.motor_abs_halls[1],
                 app_data.motor_abs_halls[2],
                 app_data.motor_abs_halls[3],
                 app_data.min_mount_halls[0],
                 app_data.min_mount_halls[1],
                 app_data.min_mount_halls[2],
                 app_data.min_mount_halls[3],
                 app_data.max_travel_range_mm);

    // 1. 执行托管的 4 路电机驱动器硬件初始化
    App_Comm_InitHardwareSequence();

    TickType_t xLastWakeTime     = xTaskGetTickCount();
    static uint8_t timer_4ms_cnt = 0;
    static uint8_t poll_cnt      = 0;

    static uint16_t pending_speed[4] = {0};
    static bool has_pending_speed[4] = {false, false, false, false};

    static Motor_Cmd_Type_t pending_cmd[4];
    static bool has_pending_cmd[4] = {false, false, false, false};

    while (1) {
        // 1. 1ms 无延迟实时推进 Modbus 接收解析与状态机释放 (ACK 收到后最快 1ms 解锁 IDLE)
        MID_Modbus_Process_1ms();

        // 2. 每 4ms 定频分频触发一次 4 级优先级发包 (250Hz 极速定频)
        if (++timer_4ms_cnt >= 4) {
            timer_4ms_cnt = 0;
            poll_cnt++;

            // 消费控制队列命令并分别归类至转速槽 (0x2001) 与命令槽 (0x2000)
            while (xQueueReceive(g_motor_ctrl_queue, &ctrl_msg, 0) == pdTRUE) {
                for (int i = 0; i < 4; i++) {
                    if (ctrl_msg.motor_mask & (1 << i)) {
                        if (ctrl_msg.cmd_type == CMD_SET_SPEED) {
                            pending_speed[i]     = ctrl_msg.speed_rpm;
                            has_pending_speed[i] = true;
                        } else {
                            pending_cmd[i]     = ctrl_msg.cmd_type;
                            has_pending_cmd[i] = true;
                        }
                    }
                }
            }

            for (int i = 0; i < 4; i++) {
                Modbus_Master_t *m = &modbus_masters[i];
                if (m->state != MODBUS_STATE_IDLE) continue;

                // Tier 1: 优先下发待更新的转速设置指令 (写 0x2001)
                if (has_pending_speed[i]) {
                    if (MID_Modbus_WriteSingleReg(m, 0x2001, pending_speed[i], Motor_Speed_Callbacks[i])) {
                        has_pending_speed[i] = false;
                    }
                }
                // Tier 2: 其次下发待更新的运行/停止控制指令 (写 0x2000)
                else if (has_pending_cmd[i]) {
                    uint16_t reg_val = 0x0009; // 默认 STOP
                    if (pending_cmd[i] == CMD_FORWARD) {
                        reg_val = 0x0001;
                    } else if (pending_cmd[i] == CMD_REVERSE) {
                        reg_val = 0x0002;
                    } else if (pending_cmd[i] == CMD_STOP) {
                        reg_val = 0x0009;
                    }

                    if (MID_Modbus_WriteSingleReg(m, 0x2000, reg_val, Motor_Cmd_Callbacks[i])) {
                        has_pending_cmd[i] = false;
                    }
                }
                // Tier 3 & Tier 4: 无高优先级控制指令时，下发周期采样
                else {
                    if (poll_cnt >= 25) {
                        // Tier 3: 每 25 帧 (100ms) 抽样读取一次 0x3004 电流
                        MID_Modbus_ReadRegs(m, 0x3004, 1, Motor_ReadCurrent_Callbacks[i]);
                    } else {
                        // Tier 4: 空闲缝隙无条件读取 0x3013 霍尔位置
                        MID_Modbus_ReadRegs(m, 0x3013, 2, Motor_ReadHall_Callbacks[i]);
                    }
                }
            }

            if (poll_cnt >= 25) {
                poll_cnt = 0;
            }
        }

        // 3. 严格 1.0ms 绝对周期调度
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
