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

// ========================== 硬件初始化写应答跟踪 ==========================

#define COMM_INIT_ALL_MASK 0x0F

static volatile uint8_t s_init_done_mask;
static volatile uint8_t s_init_ok_mask;

static void Init_Write_Callback_Generic(uint8_t motor_idx, uint8_t success)
{
    uint8_t bit = (uint8_t)(1u << motor_idx);
    if (success) {
        s_init_ok_mask |= bit;
    } else {
        s_init_ok_mask &= (uint8_t)~bit;
    }
    s_init_done_mask |= bit;
}

#define DEFINE_INIT_WRITE_CALLBACK(num, idx)                     \
    static void Motor##num##_InitWrite_Callback(uint8_t success) \
    {                                                            \
        Init_Write_Callback_Generic(idx, success);               \
    }

DEFINE_INIT_WRITE_CALLBACK(1, 0)
DEFINE_INIT_WRITE_CALLBACK(2, 1)
DEFINE_INIT_WRITE_CALLBACK(3, 2)
DEFINE_INIT_WRITE_CALLBACK(4, 3)

static const modbus_write_callback_t Motor_InitWrite_Callbacks[4] = {
    Motor1_InitWrite_Callback, Motor2_InitWrite_Callback, Motor3_InitWrite_Callback, Motor4_InitWrite_Callback};

static void App_Comm_WaitInitWrites(uint8_t pending_mask)
{
    while ((s_init_done_mask & pending_mask) != pending_mask) {
        MID_Modbus_Process_1ms();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void App_Comm_WriteRegUntilAllOk(uint16_t reg, uint16_t val, const char *name)
{
    uint8_t ok_mask      = 0;
    uint16_t retry_round = 0;

    Debug_Printf("[COMM] Init %s (0x%04X=0x%04X)\r\n", name, reg, val);

    while (ok_mask != COMM_INIT_ALL_MASK) {
        uint8_t pending_mask = ok_mask;
        s_init_done_mask     = ok_mask;
        s_init_ok_mask       = ok_mask;

        for (int i = 0; i < 4; i++) {
            if (ok_mask & (1u << i)) {
                continue;
            }
            if (MID_Modbus_WriteSingleReg(&modbus_masters[i], reg, val, Motor_InitWrite_Callbacks[i])) {
                pending_mask |= (uint8_t)(1u << i);
            }
        }

        if (pending_mask == ok_mask) {
            MID_Modbus_Process_1ms();
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        App_Comm_WaitInitWrites(pending_mask);
        ok_mask = s_init_ok_mask;

        if (ok_mask != COMM_INIT_ALL_MASK) {
            retry_round++;
            if ((retry_round % 20u) == 0u) {
                Debug_Printf("[COMM] Init %s retry %u ok_mask=0x%02X\r\n", name, retry_round, ok_mask);
            }
        }
    }
}

static void App_Comm_SwitchBaudUntilAllOk(void)
{
    uint8_t ok_mask           = 0;
    uint8_t send_at_115200[4] = {0, 0, 0, 0};
    uint16_t retry_round      = 0;

    Debug_Printf("[COMM] Init BaudRate 0x2009=7 -> 115200\r\n");

    while (ok_mask != COMM_INIT_ALL_MASK) {
        uint8_t pending_mask = ok_mask;
        s_init_done_mask     = ok_mask;
        s_init_ok_mask       = ok_mask;

        for (int i = 0; i < 4; i++) {
            bool sent;

            if (ok_mask & (1u << i)) {
                continue;
            }

            if (send_at_115200[i]) {
                /* 驱动器可能已经切到 115200，但上次应答丢失，改在 115200 重发 */
                MID_Modbus_SetMasterBaudRate(&modbus_masters[i], 115200);
                sent = MID_Modbus_WriteSingleReg(&modbus_masters[i], 0x2009, 7, Motor_InitWrite_Callbacks[i]);
            } else {
                /* 默认按 19200 下发，发完立刻切本地 115200 收驱动器应答 */
                MID_Modbus_SetMasterBaudRate(&modbus_masters[i], 19200);
                sent = MID_Modbus_WriteSingleRegThenSwitchBaud(&modbus_masters[i], 0x2009, 7, 115200, Motor_InitWrite_Callbacks[i]);
            }

            if (sent) {
                pending_mask |= (uint8_t)(1u << i);
            }
        }

        if (pending_mask == ok_mask) {
            MID_Modbus_Process_1ms();
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        App_Comm_WaitInitWrites(pending_mask);

        for (int i = 0; i < 4; i++) {
            uint8_t bit = (uint8_t)(1u << i);
            if (ok_mask & bit) {
                continue;
            }
            if (s_init_ok_mask & bit) {
                ok_mask |= bit;
            } else if (pending_mask & bit) {
                send_at_115200[i] = (uint8_t)!send_at_115200[i];
            }
        }

        if (ok_mask != COMM_INIT_ALL_MASK) {
            retry_round++;
            if ((retry_round % 20u) == 0u) {
                Debug_Printf("[COMM] Init BaudRate retry %u ok_mask=0x%02X\r\n", retry_round, ok_mask);
            }
        }
    }
}

// ========================== 485 并行 Modbus 硬件初始化序列 ==========================

static void App_Comm_InitHardwareSequence(void)
{
    /* 波特率指令也无限重试，直到 4 路都收到 115200 应答 */
    App_Comm_SwitchBaudUntilAllOk();

    /* 后续每条配置都等 4 路成功后才进入下一条 */
    struct {
        uint16_t reg;
        uint16_t val;
        const char *name;
    } init_steps[] = {
        {0x2000, 0x0007, "Fault Reset"},
        {0x200E, 0x0001, "Write Enable"},
        {0x2006, 0x0002, "Run Mode"},
        {0x2007, 0x0003, "Speed Mode"},
        {0x2001, 300, "Set Speed 300"},
        {0x2000, 0x0005, "Start Drive"}};

    int num_steps = sizeof(init_steps) / sizeof(init_steps[0]);
    for (int step = 0; step < num_steps; step++) {
        App_Comm_WriteRegUntilAllOk(init_steps[step].reg, init_steps[step].val, init_steps[step].name);
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

    TickType_t xLastWakeTime            = xTaskGetTickCount();
    static uint16_t timer_4ms_cnt       = 0;
    static uint16_t current_poll_cnt[4] = {0, 0, 0, 0};

    static int16_t pending_speed[4];
    static bool has_pending_speed[4] = {false, false, false, false};

    static Motor_Cmd_Type_t pending_cmd[4];
    static bool has_pending_cmd[4] = {false, false, false, false};

    while (1) {
        // 1. 1ms 无延迟实时推进 Modbus 接收解析与状态机释放 (ACK 收到后最快 1ms 解锁 IDLE)
        MID_Modbus_Process_1ms();

        // 2. 每 4ms 定频分频触发一次 4 级优先级发包 (250Hz 极速定频)
        if (++timer_4ms_cnt >= 4) {
            timer_4ms_cnt = 0;

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
                // Tier 3 & Tier 4: 无高优先级控制指令时，各通道独立下发周期采样
                else {
                    current_poll_cnt[i]++;
                    if (current_poll_cnt[i] >= CURRENT_POLL_INTERVAL_FRAMES) {
                        // Tier 3: 当达到抽样帧间隔时下发电流读取 (0x3004)，仅当成功下发后才复位该通道计数
                        if (MID_Modbus_ReadRegs(m, 0x3004, 1, Motor_ReadCurrent_Callbacks[i])) {
                            current_poll_cnt[i] = 0;
                        }
                    } else {
                        // Tier 4: 其余空闲缝隙无条件读取 0x3013 霍尔位置
                        MID_Modbus_ReadRegs(m, 0x3013, 2, Motor_ReadHall_Callbacks[i]);
                    }
                }
            }
        }

        // 3. 严格 1.0ms 绝对周期调度
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
