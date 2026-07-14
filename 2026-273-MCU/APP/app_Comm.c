#include "app_Comm.h"
#include "mid_modbus.h"
#include "app_pid.h"
#include <stdio.h>
#include <stdarg.h>

// 导入调试串口 5 句柄
extern UART_HandleTypeDef huart5;

// 实例化全局电机监控及队列变量
Motor_Status_t g_motor_status[4];
QueueHandle_t g_motor_ctrl_queue = NULL;

// 全局基准速度，用于 PID 调整基准
int16_t g_base_speed = 0;

// 4路电机的PID控制器结构体实例 (由 app_pid.c 提供算法底层支持)
static APP_PID_Handle_t motor_pids[4];

// 调试宏：设置为 1 开启四路电机霍尔值读取完毕后触发断点，设置为 0 恢复正常运行
#define DEBUG_HALL_BREAKPOINT 0

// ========================== 调试串口 5 格式化输出 ==========================

/**
 * @brief  调试串口 5 的 printf 重定向封装
 */
static void Debug_Printf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (len > 0) {
        HAL_UART_Transmit(&huart5, (uint8_t *)buffer, len, 100);
    }
}

// ========================== PID 同步控制计算 ==========================

/**
 * @brief  PID 同步计算，基于平均霍尔位置调节各路电机的目标速度
 * @param  base_speed 基准转速
 */
static void APP_Comm_RunPID(int16_t base_speed)
{
    if (base_speed <= 0) return;

    // 1. 计算 4 路电机的平均当前霍尔计数值
    float avg_hall = 0.0f;
    for (int i = 0; i < 4; i++) {
        avg_hall += (float)g_motor_status[i].hall_value;
    }
    avg_hall /= 4.0f;

    // 2. 依次为每个电机执行 PID 位置同步校正
    for (int i = 0; i < 4; i++) {
        // 动态将 PID 控制器的目标位置设定为 4 路电机的平均霍尔值
        APP_PID_SetTarget(&motor_pids[i], avg_hall);

        // 反馈输入当前电机的真实霍尔值，计算得出速度修偏量 delta_v
        float delta_v = APP_PID_Calc(&motor_pids[i], (float)g_motor_status[i].hall_value);

        float target_v = 0.0f;

        // 3. 根据运行方向（正向/反向）确定速度调节的极性
        if (g_motor_status[i].target_cmd == CMD_FORWARD) {
            // 正转上升段：高度低的（delta_v为正）加速，高度超前的（delta_v为负）减速
            target_v = (float)base_speed + delta_v;
        } else if (g_motor_status[i].target_cmd == CMD_REVERSE) {
            // 反转下降段：高度高的（偏前，delta_v为负）应该加速下降，故用负号取反
            target_v = (float)base_speed - delta_v;
        } else {
            target_v = 0.0f;
        }

        // 4. 进行安全防飞车速度限幅 (限制在 30 ~ 200 RPM 之间)
        if (target_v > 200.0f)
            target_v = 200.0f;
        else if (target_v < 30.0f && g_motor_status[i].target_cmd != CMD_STOP)
            target_v = 30.0f;

        // 5. 应用修偏后的目标转速
        if (g_motor_status[i].target_cmd == CMD_STOP) {
            g_motor_status[i].target_speed = 0;
        } else {
            g_motor_status[i].target_speed = (int16_t)target_v;
        }
    }

    // 6. 打印霍尔实时同步状态与调速波形 (格式适配多通道波形软件，如 VOFA+)
    Debug_Printf("H0:%u,H1:%u,H2:%u,H3:%u,Avg:%.1f,V0:%d,V1:%d,V2:%d,V3:%d\r\n",
                 g_motor_status[0].hall_value,
                 g_motor_status[1].hall_value,
                 g_motor_status[2].hall_value,
                 g_motor_status[3].hall_value,
                 avg_hall,
                 g_motor_status[0].target_speed,
                 g_motor_status[1].target_speed,
                 g_motor_status[2].target_speed,
                 g_motor_status[3].target_speed);
}

/**
 * @brief  仅用于硬件断点调试的辅助回调，不再干涉控制和触发 PID，避免影响时序
 */
static void Check_Hall_Breakpoint(uint8_t motor_idx)
{
#if DEBUG_HALL_BREAKPOINT
    static uint8_t s_hall_acquired_mask = 0;
    s_hall_acquired_mask |= (1 << motor_idx);
    if (s_hall_acquired_mask == 0x0F) {
        s_hall_acquired_mask = 0;

// 触发软断点
#if defined(__CC_ARM)
        __breakpoint(0); // Keil AC5
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
        __asm volatile("bkpt 0"); // Keil AC6
#elif defined(__GNUC__)
        __asm volatile("bkpt 0"); // GCC
#else
        volatile uint8_t break_here = 1;
        (void)break_here;
#endif
    }
#else
    (void)motor_idx;
#endif
}

// ========================== Modbus 通用回调泛型函数 ==========================

/**
 * @brief  通用电机配置回调 (推进初始化状态机)
 */
static void Motor_Config_Callback_Generic(uint8_t motor_idx, uint8_t success)
{
    if (success) {
        if (g_motor_status[motor_idx].init_step < MOTOR_INIT_STEP_DONE) {
            g_motor_status[motor_idx].init_step++;
            Debug_Printf("[INIT] Motor %d step -> %d\r\n", motor_idx, g_motor_status[motor_idx].init_step);
        }
        g_motor_status[motor_idx].retry_cnt = 0;
    } else {
        g_motor_status[motor_idx].retry_cnt++;
        Debug_Printf("[INIT] Motor %d step %d Failed, retry=%d\r\n",
                     motor_idx, g_motor_status[motor_idx].init_step, g_motor_status[motor_idx].retry_cnt);
    }
}

/**
 * @brief  通用动作命令写入回调 (确认当前运行命令)
 */
static void Motor_Cmd_Callback_Generic(uint8_t motor_idx, uint8_t success)
{
    if (success) {
        g_motor_status[motor_idx].current_cmd = g_motor_status[motor_idx].target_cmd;
        g_motor_status[motor_idx].retry_cnt   = 0;
    } else {
        g_motor_status[motor_idx].retry_cnt++;
    }
}

/**
 * @brief  通用目标速度写入回调 (确认当前速度配置)
 */
static void Motor_Speed_Callback_Generic(uint8_t motor_idx, uint8_t success)
{
    if (success) {
        g_motor_status[motor_idx].current_speed = g_motor_status[motor_idx].target_speed;
        g_motor_status[motor_idx].retry_cnt     = 0;
    } else {
        g_motor_status[motor_idx].retry_cnt++;
    }
}

/**
 * @brief  通用霍尔读取回调 (更新 32 位霍尔脉冲值)
 */
static void Motor_ReadHall_Callback_Generic(uint8_t motor_idx, uint16_t *pData, uint8_t success)
{
    if (success && pData != NULL) {
        g_motor_status[motor_idx].hall_value = ((uint32_t)pData[0] << 16) | pData[1];
        g_motor_status[motor_idx].comm_error = 0;

        Check_Hall_Breakpoint(motor_idx);
    } else {
        if (g_motor_status[motor_idx].comm_error == 0) {
            Debug_Printf("[ERR] Motor %d Comm Timeout!\r\n", motor_idx);
        }
        g_motor_status[motor_idx].comm_error = 1;
    }
}

// ========================== 回调函数批量宏展开定义 ==========================

#define DEFINE_MOTOR_CALLBACKS(num, idx)                                         \
    static void Motor##num##_Config_Callback(uint8_t success)                    \
    {                                                                            \
        Motor_Config_Callback_Generic(idx, success);                             \
    }                                                                            \
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

// 展开生成 4 路电机的具体回调函数
DEFINE_MOTOR_CALLBACKS(1, 0)
DEFINE_MOTOR_CALLBACKS(2, 1)
DEFINE_MOTOR_CALLBACKS(3, 2)
DEFINE_MOTOR_CALLBACKS(4, 3)

// ========================== 回调函数数组绑定 ==========================
static const modbus_write_callback_t Motor_Config_Callbacks[4] = {
    Motor1_Config_Callback, Motor2_Config_Callback, Motor3_Config_Callback, Motor4_Config_Callback};

static const modbus_write_callback_t Motor_Cmd_Callbacks[4] = {
    Motor1_Cmd_Callback, Motor2_Cmd_Callback, Motor3_Cmd_Callback, Motor4_Cmd_Callback};

static const modbus_write_callback_t Motor_Speed_Callbacks[4] = {
    Motor1_Speed_Callback, Motor2_Speed_Callback, Motor3_Speed_Callback, Motor4_Speed_Callback};

static const modbus_read_callback_t Motor_ReadHall_Callbacks[4] = {
    Motor1_ReadHall_Callback, Motor2_ReadHall_Callback, Motor3_ReadHall_Callback, Motor4_ReadHall_Callback};

/**
 * @brief  485 并行 Modbus 轮询与超时管理任务
 * @note   工作状态下以 20ms 为周期高频轮询读霍尔，空闲状态下（已停机且无新事件）保持无限期阻塞挂起
 */
void APP_CommTask(void *pvParameters)
{
    Motor_Ctrl_Msg_t ctrl_msg;
    uint32_t schedule_cnt  = 0;
    uint8_t read_timers[4] = {0, 0, 0, 0};

    // 1. 创建控制消息队列 (深度：10)
    g_motor_ctrl_queue = xQueueCreate(10, sizeof(Motor_Ctrl_Msg_t));

    // 2. 初始化 4 路立柱电机的同步 PID 参数
    // 比例系数 Kp = 0.5f，积分系数 Ki = 0.01f，微分系数 Kd = 0.0f
    // 设定目标值初始设为 0.0f
    // 最大速度调节修偏增量限制在 [-50.0f, 50.0f] RPM，抗积分饱和限制 20.0f
    for (int i = 0; i < 4; i++) {
        APP_PID_Init(&motor_pids[i], 0.3f, 0.005f, 0.0f, 0.0f, 50.0f, -50.0f, 20.0f);
    }

    // 3. 初始化电机监控变量为默认停机状态
    for (int i = 0; i < 4; i++) {
        g_motor_status[i].init_step  = MOTOR_INIT_STEP_WRITE_ENABLE;
        g_motor_status[i].hall_value = 0;
        g_motor_status[i].comm_error = 0;
        g_motor_status[i].retry_cnt  = 0;

        // 初始的目标控制动作为停止，目标速度为 0 RPM
        g_motor_status[i].target_cmd   = CMD_STOP;
        g_motor_status[i].target_speed = 0;

        // 初始真实状态设为停止，确保不引发无谓的更新动作
        g_motor_status[i].current_cmd   = CMD_STOP;
        g_motor_status[i].current_speed = 0;
    }

    Debug_Printf("[SYS] APP_CommTask Started, Waiting for Control Msg...\r\n");

    while (1) {
        // 4. 动态阻塞机制：判定当前是工作状态还是纯空闲状态
        TickType_t block_time = pdMS_TO_TICKS(1); // 默认工作状态下，1ms 唤醒一次，维持 Modbus 超时状态机精度
        bool all_idle         = true;

        for (int i = 0; i < 4; i++) {
            // 如果存在电机还未初始化完成，或者目标命令非停止，或者驱动器真实状态还未停下，则判断为工作中
            if (g_motor_status[i].init_step < MOTOR_INIT_STEP_DONE ||
                g_motor_status[i].target_cmd != CMD_STOP ||
                g_motor_status[i].current_cmd != CMD_STOP) {
                all_idle = false;
                break;
            }
        }

        if (all_idle) {
            block_time = portMAX_DELAY; // 纯空闲无更新任务，无限期阻塞死等新消息，0% CPU 唤醒
        } else {
            block_time = pdMS_TO_TICKS(1); // 工作中，1ms 快速唤醒推进状态机
        }

        // 5. 阻塞接收队列消息 (空闲时无限死等，工作时 1ms 超时快速轮询)
        if (xQueueReceive(g_motor_ctrl_queue, &ctrl_msg, block_time) == pdTRUE) {
            g_base_speed = ctrl_msg.speed_rpm;
            Debug_Printf("[MSG] Ctrl Command Received: Cmd=%d, Speed=%d\r\n", ctrl_msg.cmd_type, ctrl_msg.speed_rpm);

            // 收到控制台下发的新指令，同步更新 4 路电机的控制目标
            for (int i = 0; i < 4; i++) {
                // 如果运行方向/启停动作发生了改变，立刻清空该通道 PID 积分项，防止带入历史误差
                if (g_motor_status[i].target_cmd != ctrl_msg.cmd_type) {
                    APP_PID_Clear(&motor_pids[i]);
                }

                g_motor_status[i].target_cmd   = ctrl_msg.cmd_type;
                g_motor_status[i].target_speed = ctrl_msg.speed_rpm;
            }
        }

        // 6. 推进底层 4 路串口 Modbus 主站的状态机 (非空闲时时基精度维持 1ms)
        if (!all_idle) {
            MID_Modbus_Process_1ms();
        }

        // 7. 工作状态下以 20ms 的合理节拍进行 Modbus 并行总线调度
        schedule_cnt++;
        if (schedule_cnt >= 20) {
            schedule_cnt = 0;

            // ================== PID 同步计算时基 (强制每 100ms 触发一次，彻底切断凑数死锁) ==================
            static uint8_t pid_run_timer = 0;
            pid_run_timer++;
            if (pid_run_timer >= 5) // 5 * 20ms = 100ms
            {
                pid_run_timer = 0;

                // 只有在电机配置完且正在处于运动控制阶段时，才强制执行 PID 同步算法并打印波形
                if (!all_idle) {
                    APP_Comm_RunPID(g_base_speed);
                }
            }

            for (int i = 0; i < 4; i++) {
                Modbus_Master_t *m = &modbus_masters[i];

                if (m->state == MODBUS_STATE_IDLE) {
                    // === 运行阶段 1：各电机逐步执行 5 步初始化配置流程 ===
                    if (g_motor_status[i].init_step < MOTOR_INIT_STEP_DONE) {
                        switch (g_motor_status[i].init_step) {
                            case MOTOR_INIT_STEP_WRITE_ENABLE:
                                MID_Modbus_WriteSingleReg(m, 0x200E, 0x0000, Motor_Config_Callbacks[i]);
                                break;
                            case MOTOR_INIT_STEP_RUN_MODE:
                                MID_Modbus_WriteSingleReg(m, 0x2006, 0x0002, Motor_Config_Callbacks[i]);
                                break;
                            case MOTOR_INIT_STEP_SPEED_MODE:
                                MID_Modbus_WriteSingleReg(m, 0x2007, 0x0003, Motor_Config_Callbacks[i]);
                                break;
                            case MOTOR_INIT_STEP_SET_SPEED:
                                MID_Modbus_WriteSingleReg(m, 0x2001, 0, Motor_Config_Callbacks[i]);
                                break;
                            case MOTOR_INIT_STEP_START_RUN:
                                MID_Modbus_WriteSingleReg(m, 0x2000, 0x0005, Motor_Config_Callbacks[i]);
                                break;
                            default:
                                break;
                        }
                    }
                    // === 运行阶段 2：初始化完毕，进入应用层事件控制与周期霍尔轮询 ===
                    else {
                        // 2.1 优先响应控制需求更新 (如果目标速度与实际配置不一致，先发新速度)
                        if (g_motor_status[i].target_speed != g_motor_status[i].current_speed) {
                            MID_Modbus_WriteSingleReg(m, 0x2001, g_motor_status[i].target_speed, Motor_Speed_Callbacks[i]);
                        }
                        // 2.2 如果目标命令不一致，发送新指令更新 (正转/反转/停机)
                        else if (g_motor_status[i].target_cmd != g_motor_status[i].current_cmd) {
                            uint16_t cmd_val = 0x0005; // 默认停机
                            if (g_motor_status[i].target_cmd == CMD_FORWARD)
                                cmd_val = 0x0001;
                            else if (g_motor_status[i].target_cmd == CMD_REVERSE)
                                cmd_val = 0x0002;
                            else if (g_motor_status[i].target_cmd == CMD_STOP)
                                cmd_val = 0x0005;

                            MID_Modbus_WriteSingleReg(m, 0x2000, cmd_val, Motor_Cmd_Callbacks[i]);
                        }
                        // 2.3 在运行状态处于稳定且无新写更新需求时，每隔 100ms 周期性读取一次霍尔脉冲
                        else {
                            read_timers[i]++;
                            if (read_timers[i] >= 5) // 5 * 20ms = 100ms
                            {
                                read_timers[i] = 0;
                                MID_Modbus_ReadRegs(m, 0x3013, 2, Motor_ReadHall_Callbacks[i]);
                            }
                        }
                    }
                }
            }
        }
    }
}
