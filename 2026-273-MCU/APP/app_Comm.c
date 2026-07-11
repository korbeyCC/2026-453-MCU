#include "app_Comm.h"
#include "mid_modbus.h"

// 实例化全局电机监控变量
Motor_Status_t g_motor_status[4];

// 调试宏：设置为 1 开启四路电机霍尔值读取完毕后触发断点，设置为 0 恢复正常运行
#define DEBUG_HALL_BREAKPOINT   1

#if DEBUG_HALL_BREAKPOINT
/**
 * @brief  检查四路电机是否均成功获取过霍尔值，并在满足条件时触发断点（每轮触发一次）
 * @param  motor_idx  电机索引 (0-3)
 */
static void Check_Hall_Breakpoint(uint8_t motor_idx)
{
    static uint8_t s_hall_acquired_mask = 0;
    
    s_hall_acquired_mask |= (1 << motor_idx);
    if (s_hall_acquired_mask == 0x0F) {
        // 这一轮四路电机均已获取到霍尔值，重置掩码以准备下一轮再次触发
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
        (void)break_here; // 可以在此行手动添加断点
        #endif
    }
}
#else
#define Check_Hall_Breakpoint(motor_idx) ((void)0)
#endif

// ========================== 1号电机回调函数 ==========================
static void Motor1_Config_Callback(uint8_t success)
{
    if (success) {
        if (g_motor_status[0].init_step < MOTOR_INIT_STEP_DONE) {
            g_motor_status[0].init_step++;
        }
        g_motor_status[0].retry_cnt = 0;
    } else {
        g_motor_status[0].retry_cnt++;
    }
}

static void Motor1_ReadHall_Callback(uint16_t *pData, uint8_t success)
{
    if (success && pData != NULL) {
        // 读取霍尔脉冲：高16位由 pData[0] 返回，低16位由 pData[1] 返回，合并为 32 位霍尔脉冲
        g_motor_status[0].hall_value = ((uint32_t)pData[0] << 16) | pData[1];
        g_motor_status[0].comm_error = 0;
        Check_Hall_Breakpoint(0);
    } else {
        g_motor_status[0].comm_error = 1; // 标记通讯故障
    }
}

// ========================== 2号电机回调函数 ==========================
static void Motor2_Config_Callback(uint8_t success)
{
    if (success) {
        if (g_motor_status[1].init_step < MOTOR_INIT_STEP_DONE) {
            g_motor_status[1].init_step++;
        }
        g_motor_status[1].retry_cnt = 0;
    } else {
        g_motor_status[1].retry_cnt++;
    }
}

static void Motor2_ReadHall_Callback(uint16_t *pData, uint8_t success)
{
    if (success && pData != NULL) {
        g_motor_status[1].hall_value = ((uint32_t)pData[0] << 16) | pData[1];
        g_motor_status[1].comm_error = 0;
        Check_Hall_Breakpoint(1);
    } else {
        g_motor_status[1].comm_error = 1;
    }
}

// ========================== 3号电机回调函数 ==========================
static void Motor3_Config_Callback(uint8_t success)
{
    if (success) {
        if (g_motor_status[2].init_step < MOTOR_INIT_STEP_DONE) {
            g_motor_status[2].init_step++;
        }
        g_motor_status[2].retry_cnt = 0;
    } else {
        g_motor_status[2].retry_cnt++;
    }
}

static void Motor3_ReadHall_Callback(uint16_t *pData, uint8_t success)
{
    if (success && pData != NULL) {
        g_motor_status[2].hall_value = ((uint32_t)pData[0] << 16) | pData[1];
        g_motor_status[2].comm_error = 0;
        Check_Hall_Breakpoint(2);
    } else {
        g_motor_status[2].comm_error = 1;
    }
}

// ========================== 4号电机回调函数 ==========================
static void Motor4_Config_Callback(uint8_t success)
{
    if (success) {
        if (g_motor_status[3].init_step < MOTOR_INIT_STEP_DONE) {
            g_motor_status[3].init_step++;
        }
        g_motor_status[3].retry_cnt = 0;
    } else {
        g_motor_status[3].retry_cnt++;
    }
}

static void Motor4_ReadHall_Callback(uint16_t *pData, uint8_t success)
{
    if (success && pData != NULL) {
        g_motor_status[3].hall_value = ((uint32_t)pData[0] << 16) | pData[1];
        g_motor_status[3].comm_error = 0;
        Check_Hall_Breakpoint(3);
    } else {
        g_motor_status[3].comm_error = 1;
    }
}

// 整理回调函数指针数组，便于通过循环下标处理
static const modbus_write_callback_t Motor_Config_Callbacks[4] = {
    Motor1_Config_Callback,
    Motor2_Config_Callback,
    Motor3_Config_Callback,
    Motor4_Config_Callback
};

static const modbus_read_callback_t Motor_ReadHall_Callbacks[4] = {
    Motor1_ReadHall_Callback,
    Motor2_ReadHall_Callback,
    Motor3_ReadHall_Callback,
    Motor4_ReadHall_Callback
};

/**
 * @brief  485 并行 Modbus 轮询与超时管理任务
 * @note   每 1ms 唤醒一次，推进底层状态机计时，并按照 20ms 的通信时间节拍异步发送指令
 */
void APP_CommTask(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    uint32_t schedule_cnt = 0;
    
    // 初始化监控变量状态
    for (int i = 0; i < 4; i++) {
        g_motor_status[i].init_step   = MOTOR_INIT_STEP_WRITE_ENABLE;
        g_motor_status[i].hall_value  = 0;
        g_motor_status[i].comm_error  = 0;
        g_motor_status[i].retry_cnt   = 0;
        g_motor_status[i].read_timer  = 0;
    }
    
    while (1)
    {
        // 1. 推进底层的 Modbus 状态机及超时逻辑 (精度：1ms)
        MID_Modbus_Process_1ms();
        
        // 2. 串口调度中心：每 20ms 处理一次 Modbus 物理发送，防止总线拥堵
        schedule_cnt++;
        if (schedule_cnt >= 20)
        {
            schedule_cnt = 0;
            
            for (int i = 0; i < 4; i++)
            {
                Modbus_Master_t *m = &modbus_masters[i];
                
                // 只有当对应串口的 Modbus 主站处于 IDLE 空闲状态时，才可以安全发起下一次发送
                if (m->state == MODBUS_STATE_IDLE)
                {
                    // === 电机驱动器初始化步骤管理 ===
                    if (g_motor_status[i].init_step < MOTOR_INIT_STEP_DONE)
                    {
                        switch (g_motor_status[i].init_step)
                        {
                            case MOTOR_INIT_STEP_WRITE_ENABLE:
                                // 步骤 1: 0x200E 功能码写使能，写入 0x0000 
                                MID_Modbus_WriteSingleReg(m, 0x200E, 0x0000, Motor_Config_Callbacks[i]);
                                break;
                                
                            case MOTOR_INIT_STEP_RUN_MODE:
                                // 步骤 2: 0x2006 运行命令选通讯方式，写入 0x0002 
                                MID_Modbus_WriteSingleReg(m, 0x2006, 0x0002, Motor_Config_Callbacks[i]);
                                break;
                                
                            case MOTOR_INIT_STEP_SPEED_MODE:
                                // 步骤 3: 0x2007 速度命令选通讯方式，写入 0x0003 
                                MID_Modbus_WriteSingleReg(m, 0x2007, 0x0003, Motor_Config_Callbacks[i]);
                                break;
                                
                            case MOTOR_INIT_STEP_SET_SPEED:
                                // 步骤 4: 0x2001 设定电机目标转速为 150 RPM (写入 0x0096)
                                MID_Modbus_WriteSingleReg(m, 0x2001, 0x0096, Motor_Config_Callbacks[i]);
                                break;
                                
                            case MOTOR_INIT_STEP_START_RUN:
                                // 步骤 5: 0x2000 启动正转，写入 0x0001
                                MID_Modbus_WriteSingleReg(m, 0x2000, 0x0001, Motor_Config_Callbacks[i]);
                                break;
                                
                            default:
                                break;
                        }
                    }
                    else
                    {
                        // === 电机已正转运行，启动周期性读霍尔轮询 (频率：每 100ms 读取一次) ===
                        g_motor_status[i].read_timer++;
                        if (g_motor_status[i].read_timer >= 5) // 5 * 20ms = 100ms
                        {
                            g_motor_status[i].read_timer = 0;
                            // 0x3013 起始地址，读取 2 个寄存器 (即 32 位的霍尔计数值)
                            MID_Modbus_ReadRegs(m, 0x3013, 2, Motor_ReadHall_Callbacks[i]);
                        }
                    }
                }
            }
        }
        
        // 1ms 任务唤醒周期
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(1));
    }
}
