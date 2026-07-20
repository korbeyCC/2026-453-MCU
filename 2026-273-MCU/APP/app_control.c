#include "app_control.h"
#include "mid_signal.h"
#include "mid_led.h"
#include "mid_modbus.h"
#include "app_Comm.h"
#include "app_Data.h"

// 实例化全局控制上下文
Sys_Ctrl_Context_t g_sys_context;

// 4路电机的PID控制实例
static APP_PID_Handle_t motor_pids[4];

// ========================== 独立无副作用高度波形打印 ==========================

/**
 * @brief  输出当前 4 路绝对高度和 PID 目标转速的监控波形
 */
static void APP_Control_DebugPrint(void)
{
    float avg_travel = 0.0f;
    float travel_rel[4];
    
    // 计算各立柱当前相对行程和平均相对位移
    for (int i = 0; i < 4; i++) {
        travel_rel[i] = (float)(g_sys_context.g_motor_status[i].current_abs_hall - app_data.min_mount_halls[i]);
        avg_travel += travel_rel[i];
    }
    avg_travel /= 4.0f;
    
    Debug_Printf("H0:%d,H1:%d,H2:%d,H3:%d,AvgTravel:%.1f,V0:%d,V1:%d,V2:%d,V3:%d\r\n",
                 g_sys_context.g_motor_status[0].current_abs_hall,
                 g_sys_context.g_motor_status[1].current_abs_hall,
                 g_sys_context.g_motor_status[2].current_abs_hall,
                 g_sys_context.g_motor_status[3].current_abs_hall,
                 avg_travel,
                 g_sys_context.g_motor_status[0].target_speed,
                 g_sys_context.g_motor_status[1].target_speed,
                 g_sys_context.g_motor_status[2].target_speed,
                 g_sys_context.g_motor_status[3].target_speed);
}

// ========================== 事件驱动型 PID 控制 ==========================

/**
 * @brief  PID 同步计算，基于平均相对行程（Travel）调节各路电机的目标速度
 * @param  base_speed 基准转速
 */
static void APP_Control_RunPID(int16_t base_speed)
{
    if (base_speed <= 0) return;
    
    float avg_travel = 0.0f;
    float travel_rel[4];
    
    // 1. 计算各个立柱的当前相对行程位移
    for (int i = 0; i < 4; i++) {
        travel_rel[i] = (float)(g_sys_context.g_motor_status[i].current_abs_hall - app_data.min_mount_halls[i]);
        avg_travel += travel_rel[i];
    }
    avg_travel /= 4.0f;
    
    // 2. 对每个通道单独计算 PID 同步位置修正
    for (int i = 0; i < 4; i++)
    {
        APP_PID_SetTarget(&motor_pids[i], avg_travel);
        float delta_v = APP_PID_Calc(&motor_pids[i], travel_rel[i]);
        float target_v = 0.0f;
        
        if (g_sys_context.g_motor_status[i].target_cmd == CMD_FORWARD) {
            target_v = (float)base_speed + delta_v;
        } else if (g_sys_context.g_motor_status[i].target_cmd == CMD_REVERSE) {
            target_v = (float)base_speed - delta_v;
        } else {
            target_v = 0.0f;
        }
        
        // 限制调速范围在 30 ~ 200 RPM
        if (target_v > 200.0f) target_v = 200.0f;
        else if (target_v < 30.0f && g_sys_context.g_motor_status[i].target_cmd != CMD_STOP) target_v = 30.0f;
        
        if (g_sys_context.g_motor_status[i].target_cmd == CMD_STOP) {
            g_sys_context.g_motor_status[i].target_speed = 0;
        } else {
            g_sys_context.g_motor_status[i].target_speed = (int16_t)target_v;
        }
    }
}

// ========================== 核心业务控制任务 ==========================

/**
 * @brief 核心业务控制任务
 * @note  使用高层状态机驱动流转，采用事件驱动式 PID 与解耦波形打印，最大化防范控制时滞
 */
void APP_ControlTask(void *pvParameters)
{
    MID_SIGNAL_Msg sig_msg;

    // 1. 初始化系统状态机与 PID
    g_sys_context.system_step = SYS_STEP_Boot;
    g_sys_context.event_group = NULL;
    g_sys_context.base_speed  = 0;

    for (int i = 0; i < 4; i++) {
        APP_PID_Init(&motor_pids[i], 0.5f, 0.01f, 0.0f, 0.0f, 50.0f, -50.0f, 20.0f);
    }

    // 2. 初始化中间层灯带控制
    MID_LED_Init();

    while (1) {
        bool has_event = false;
        
        if (g_sys_context.system_step >= SYS_STEP_READY && 
            g_sys_context.system_step != SYS_STEP_TOTAL_DONE && 
            g_sys_context.system_step != SYS_STEP_TUNE_DONE)
        {
            TickType_t key_wait = (g_sys_context.system_step == SYS_STEP_READY) ? pdMS_TO_TICKS(50) : 0;
            has_event = MID_Signal_GetEvent(&sig_msg, key_wait);
        }

        // 3. 全局控制流大 switch
        switch (g_sys_context.system_step)
        {
            // === Boot 引导状态 ===
            case SYS_STEP_Boot:
            {
                // 创建 FreeRTOS 事件标志组
                g_sys_context.event_group = xEventGroupCreate();
                xEventGroupClearBits(g_sys_context.event_group, 0xFFFFFF);
                
                g_sys_context.system_step = SYS_STEP_INIT_WRITE_ENABLE;
                g_sys_context.base_speed  = 0;
                
                // 【控制层自主维护 target】
                for (int i = 0; i < 4; i++) {
                    g_sys_context.g_motor_status[i].target_cmd = CMD_INIT_WRITE_ENABLE;
                }
                
                Motor_Ctrl_Msg_t ctrl_msg = {CMD_INIT_WRITE_ENABLE, 0x0F, 0};
                xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);
                break;
            }

            // === 初始化 1 步：写使能 ===
            case SYS_STEP_INIT_WRITE_ENABLE:
            {
                xEventGroupWaitBits(g_sys_context.event_group, ALL_CMD_EVENTS_READY, pdTRUE, pdTRUE, portMAX_DELAY);
                
                g_sys_context.system_step = SYS_STEP_INIT_RUN_MODE;
                
                // 【控制层自主维护 target】
                for (int i = 0; i < 4; i++) {
                    g_sys_context.g_motor_status[i].target_cmd = CMD_INIT_RUN_MODE;
                }
                
                Motor_Ctrl_Msg_t ctrl_msg = {CMD_INIT_RUN_MODE, 0x0F, 0};
                xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);
                break;
            }

            // === 初始化 2 步：运行模式 ===
            case SYS_STEP_INIT_RUN_MODE:
            {
                xEventGroupWaitBits(g_sys_context.event_group, ALL_CMD_EVENTS_READY, pdTRUE, pdTRUE, portMAX_DELAY);
                
                g_sys_context.system_step = SYS_STEP_INIT_SPEED_MODE;
                
                // 【控制层自主维护 target】
                for (int i = 0; i < 4; i++) {
                    g_sys_context.g_motor_status[i].target_cmd = CMD_INIT_SPEED_MODE;
                }
                
                Motor_Ctrl_Msg_t ctrl_msg = {CMD_INIT_SPEED_MODE, 0x0F, 0};
                xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);
                break;
            }

            // === 初始化 3 步：速度模式 ===
            case SYS_STEP_INIT_SPEED_MODE:
            {
                xEventGroupWaitBits(g_sys_context.event_group, ALL_CMD_EVENTS_READY, pdTRUE, pdTRUE, portMAX_DELAY);
                
                g_sys_context.system_step = SYS_STEP_INIT_SET_SPEED;
                
                // 【控制层自主维护 target】
                for (int i = 0; i < 4; i++) {
                    g_sys_context.g_motor_status[i].target_cmd = CMD_INIT_SET_SPEED;
                    g_sys_context.g_motor_status[i].target_speed = 0;
                }
                
                Motor_Ctrl_Msg_t ctrl_msg = {CMD_INIT_SET_SPEED, 0x0F, 0};
                xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);
                break;
            }

            // === 初始化 4 步：目标速度 0 ===
            case SYS_STEP_INIT_SET_SPEED:
            {
                xEventGroupWaitBits(g_sys_context.event_group, ALL_CMD_EVENTS_READY, pdTRUE, pdTRUE, portMAX_DELAY);
                
                g_sys_context.system_step = SYS_STEP_INIT_START_RUN;
                
                // 【控制层自主维护 target】
                for (int i = 0; i < 4; i++) {
                    g_sys_context.g_motor_status[i].target_cmd = CMD_INIT_START_RUN;
                }
                
                Motor_Ctrl_Msg_t ctrl_msg = {CMD_INIT_START_RUN, 0x0F, 0};
                xQueueSend(g_motor_ctrl_queue, &ctrl_msg, 0);
                break;
            }

            // === 初始化 5 步：驱动器开启运行 ===
            case SYS_STEP_INIT_START_RUN:
            {
                xEventGroupWaitBits(g_sys_context.event_group, ALL_CMD_EVENTS_READY, pdTRUE, pdTRUE, portMAX_DELAY);
                
                g_sys_context.system_step = SYS_STEP_READY; // 配置完全部通过，进入就绪
                Debug_Printf("[SYS] System Initialization Done! State -> READY\r\n");
                
                // 双色灯带提示就绪
                MID_LED_Write(MID_LED_1, true);
                MID_LED_Write(MID_LED_2, false);
                
                MID_SIGNAL_Msg dummy_msg;
                while (MID_Signal_GetEvent(&dummy_msg, 0) == pdTRUE);
                break;
            }

            // === READY 状态：就绪待命 ===
            case SYS_STEP_READY:
            {
                if (has_event && sig_msg.event == MID_SIGNAL_EVT_TRIGGER)
                {
                    Motor_Ctrl_Msg_t speed_msg;
                    Motor_Ctrl_Msg_t cmd_msg;
                    bool action_valid = false;

                    switch (sig_msg.signal_id)
                    {
                        case MID_SIGNAL_REMOT_3: // 遥控下行
                            MID_LED_Write(MID_LED_1, false);
                            MID_LED_Write(MID_LED_2, false);

                            g_sys_context.base_speed = 100; // 设定全局基准速度 100 RPM

                            speed_msg.cmd_type   = CMD_INIT_SET_SPEED;
                            speed_msg.motor_mask = 0x0F;
                            speed_msg.speed_rpm  = 100;

                            cmd_msg.cmd_type     = CMD_REVERSE;
                            cmd_msg.motor_mask   = 0x0F;
                            cmd_msg.speed_rpm    = 0;

                            xEventGroupClearBits(g_sys_context.event_group, ALL_CMD_EVENTS_READY | ALL_SPEED_EVENTS_READY);
                            g_sys_context.system_step = SYS_STEP_TOTAL_REVERSE;
                            
                            // 在起跑瞬间，锁存当前高度状态为绝对零差基准，并写入 target_cmd
                            for (int i = 0; i < 4; i++) {
                                g_sys_context.g_motor_status[i].start_drive_hall = (int32_t)g_sys_context.g_motor_status[i].hall_value;
                                g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                                g_sys_context.g_motor_status[i].target_cmd       = CMD_REVERSE;
                                g_sys_context.g_motor_status[i].target_speed     = 100;
                            }
                            
                            xQueueSend(g_motor_ctrl_queue, &speed_msg, 0);
                            xQueueSend(g_motor_ctrl_queue, &cmd_msg, 0);
                            action_valid = true;
                            break;

                        case MID_SIGNAL_REMOT_4: // 遥控上行
                            MID_LED_Write(MID_LED_1, true);
                            MID_LED_Write(MID_LED_2, true);

                            g_sys_context.base_speed = 100; // 设定全局基准速度 100 RPM

                            speed_msg.cmd_type   = CMD_INIT_SET_SPEED;
                            speed_msg.motor_mask = 0x0F;
                            speed_msg.speed_rpm  = 100;

                            cmd_msg.cmd_type     = CMD_FORWARD;
                            cmd_msg.motor_mask   = 0x0F;
                            cmd_msg.speed_rpm    = 0;

                            xEventGroupClearBits(g_sys_context.event_group, ALL_CMD_EVENTS_READY | ALL_SPEED_EVENTS_READY);
                            g_sys_context.system_step = SYS_STEP_TOTAL_FORWARD;
                            
                            for (int i = 0; i < 4; i++) {
                                g_sys_context.g_motor_status[i].start_drive_hall = (int32_t)g_sys_context.g_motor_status[i].hall_value;
                                g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                                g_sys_context.g_motor_status[i].target_cmd       = CMD_FORWARD;
                                g_sys_context.g_motor_status[i].target_speed     = 100;
                            }
                            
                            xQueueSend(g_motor_ctrl_queue, &speed_msg, 0);
                            xQueueSend(g_motor_ctrl_queue, &cmd_msg, 0);
                            action_valid = true;
                            break;

                        case MID_SIGNAL_BUTON_DW: // 物理下行
                            g_sys_context.base_speed = 100;

                            speed_msg.cmd_type   = CMD_INIT_SET_SPEED;
                            speed_msg.motor_mask = 0x0F;
                            speed_msg.speed_rpm  = 100;

                            cmd_msg.cmd_type     = CMD_REVERSE;
                            cmd_msg.motor_mask   = 0x0F;
                            cmd_msg.speed_rpm    = 0;

                            xEventGroupClearBits(g_sys_context.event_group, ALL_CMD_EVENTS_READY | ALL_SPEED_EVENTS_READY);
                            g_sys_context.system_step = SYS_STEP_TOTAL_REVERSE;
                            
                            for (int i = 0; i < 4; i++) {
                                g_sys_context.g_motor_status[i].start_drive_hall = (int32_t)g_sys_context.g_motor_status[i].hall_value;
                                g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                                g_sys_context.g_motor_status[i].target_cmd       = CMD_REVERSE;
                                g_sys_context.g_motor_status[i].target_speed     = 100;
                            }
                            
                            xQueueSend(g_motor_ctrl_queue, &speed_msg, 0);
                            xQueueSend(g_motor_ctrl_queue, &cmd_msg, 0);
                            action_valid = true;
                            break;

                        case MID_SIGNAL_BUTON_UP: // 物理上行
                            g_sys_context.base_speed = 100;

                            speed_msg.cmd_type   = CMD_INIT_SET_SPEED;
                            speed_msg.motor_mask = 0x0F;
                            speed_msg.speed_rpm  = 100;

                            cmd_msg.cmd_type     = CMD_FORWARD;
                            cmd_msg.motor_mask   = 0x0F;
                            cmd_msg.speed_rpm    = 0;

                            xEventGroupClearBits(g_sys_context.event_group, ALL_CMD_EVENTS_READY | ALL_SPEED_EVENTS_READY);
                            g_sys_context.system_step = SYS_STEP_TOTAL_FORWARD;
                            
                            for (int i = 0; i < 4; i++) {
                                g_sys_context.g_motor_status[i].start_drive_hall = (int32_t)g_sys_context.g_motor_status[i].hall_value;
                                g_sys_context.g_motor_status[i].base_abs_hall    = g_sys_context.g_motor_status[i].current_abs_hall;
                                g_sys_context.g_motor_status[i].target_cmd       = CMD_FORWARD;
                                g_sys_context.g_motor_status[i].target_speed     = 100;
                            }
                            
                            xQueueSend(g_motor_ctrl_queue, &speed_msg, 0);
                            xQueueSend(g_motor_ctrl_queue, &cmd_msg, 0);
                            action_valid = true;
                            break;

                        default:
                            break;
                    }

                    if (action_valid)
                    {
                        MID_SIGNAL_Msg dummy_msg;
                        while (MID_Signal_GetEvent(&dummy_msg, 0) == pdTRUE);
                    }
                }
                break;
            }

            // === 起跑过度阶段 ===
            case SYS_STEP_TOTAL_FORWARD:
            case SYS_STEP_TOTAL_REVERSE:
            {
                if (has_event && sig_msg.event == MID_SIGNAL_EVT_TRIGGER)
                {
                    // 【控制层自主维护 target】
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                    }
                    
                    Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, 0x0F, 0};
                    xEventGroupClearBits(g_sys_context.event_group, ALL_READ_EVENTS_READY);
                    g_sys_context.system_step = SYS_STEP_TOTAL_DONE;
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, 0);

                    MID_SIGNAL_Msg dummy_msg;
                    while (MID_Signal_GetEvent(&dummy_msg, 0) == pdTRUE);
                }
                else
                {
                    EventBits_t uxBits = xEventGroupWaitBits(g_sys_context.event_group,
                                                            ALL_CMD_EVENTS_READY | ALL_SPEED_EVENTS_READY,
                                                            pdTRUE,
                                                            pdTRUE,
                                                            pdMS_TO_TICKS(10));
                                                            
                    if ((uxBits & (ALL_CMD_EVENTS_READY | ALL_SPEED_EVENTS_READY)) == (ALL_CMD_EVENTS_READY | ALL_SPEED_EVENTS_READY))
                    {
                        xEventGroupClearBits(g_sys_context.event_group, ALL_READ_EVENTS_READY);
                        g_sys_context.system_step = SYS_STEP_TOTAL_RUNNING;
                        Debug_Printf("[SYS] Motor running, startup events synchronized.\r\n");

                        // 下发首帧高度读取
                        Motor_Ctrl_Msg_t read_msg = {CMD_READ_HALL, 0x0F, 0};
                        xQueueSend(g_motor_ctrl_queue, &read_msg, 0);
                    }
                }
                break;
            }

            // === 核心运行调速流水线阶段 ===
            case SYS_STEP_TOTAL_RUNNING:
            {
                if (has_event && sig_msg.event == MID_SIGNAL_EVT_TRIGGER)
                {
                    // 【控制层自主维护 target】
                    for (int i = 0; i < 4; i++) {
                        g_sys_context.g_motor_status[i].target_cmd   = CMD_STOP;
                        g_sys_context.g_motor_status[i].target_speed = 0;
                    }
                    
                    Motor_Ctrl_Msg_t stop_msg = {CMD_STOP, 0x0F, 0};
                    xEventGroupClearBits(g_sys_context.event_group, ALL_READ_EVENTS_READY);
                    g_sys_context.system_step = SYS_STEP_TOTAL_DONE;
                    xQueueSend(g_motor_ctrl_queue, &stop_msg, 0);

                    MID_SIGNAL_Msg dummy_msg;
                    while (MID_Signal_GetEvent(&dummy_msg, 0) == pdTRUE);
                }
                else
                {
                    EventBits_t uxBits = xEventGroupWaitBits(g_sys_context.event_group,
                                                            ALL_READ_EVENTS_READY,
                                                            pdTRUE,
                                                            pdTRUE,
                                                            pdMS_TO_TICKS(10));
                                                            
                    if ((uxBits & ALL_READ_EVENTS_READY) == ALL_READ_EVENTS_READY)
                    {
                        // 1. 控制层解算绝对高度
                        for (int i = 0; i < 4; i++)
                        {
                            int32_t drive_relative_hall = (int32_t)g_sys_context.g_motor_status[i].hall_value;
                            g_sys_context.g_motor_status[i].current_abs_hall = 
                                g_sys_context.g_motor_status[i].base_abs_hall + (drive_relative_hall - g_sys_context.g_motor_status[i].start_drive_hall);
                        }

                        // 2. 执行事件驱动的 PID 计算，修偏内存 target_speed (改用上下文基准速度)
                        APP_Control_RunPID(g_sys_context.base_speed);

                        // 3. 紧随其后在外部运行波形打印
                        APP_Control_DebugPrint();

                        // 4. 将新算出的 4 路转速通过消息队列打包下发到底层进行写速度动作
                        for (int i = 0; i < 4; i++)
                        {
                            Motor_Ctrl_Msg_t speed_msg;
                            speed_msg.cmd_type   = CMD_INIT_SET_SPEED;
                            speed_msg.motor_mask = (1 << i);
                            speed_msg.speed_rpm  = g_sys_context.g_motor_status[i].target_speed;
                            xQueueSend(g_motor_ctrl_queue, &speed_msg, 0);
                        }

                        // 5. 不等待调速应答，立刻发起下一次高度读取，让流水线持续高速运转
                        Motor_Ctrl_Msg_t read_msg = {CMD_READ_HALL, 0x0F, 0};
                        xQueueSend(g_motor_ctrl_queue, &read_msg, 0);
                    }
                }
                break;
            }

            // === 停机归档阶段 ===
            case SYS_STEP_TOTAL_DONE:
            case SYS_STEP_TUNE_DONE:
            {
                EventBits_t uxBits = xEventGroupGetBits(g_sys_context.event_group);
                
                if ((uxBits & ALL_READ_EVENTS_READY) != ALL_READ_EVENTS_READY)
                {
                    uint8_t unread_mask = 0;
                    for (int i = 0; i < 4; i++)
                    {
                        if (!(uxBits & READ_EVENT_BIT(i)))
                        {
                            unread_mask |= (1 << i);
                        }
                    }
                    
                    if (unread_mask != 0)
                    {
                        Motor_Ctrl_Msg_t read_msg = {CMD_READ_HALL, unread_mask, 0};
                        xQueueSend(g_motor_ctrl_queue, &read_msg, 0);
                    }
                    
                    xEventGroupWaitBits(g_sys_context.event_group, ALL_READ_EVENTS_READY, pdFALSE, pdTRUE, pdMS_TO_TICKS(50));
                }
                else
                {
                    // 统一解算最终高度
                    for (int i = 0; i < 4; i++)
                    {
                        int32_t drive_relative_hall = (int32_t)g_sys_context.g_motor_status[i].hall_value;
                        g_sys_context.g_motor_status[i].current_abs_hall = 
                            g_sys_context.g_motor_status[i].base_abs_hall + (drive_relative_hall - g_sys_context.g_motor_status[i].start_drive_hall);
                    }

                    // 存 Flash
                    for (int i = 0; i < 4; i++) {
                        app_data.motor_abs_halls[i] = g_sys_context.g_motor_status[i].current_abs_hall;
                    }
                    
                    EEPROMSet = 1;
                    
                    // 重归 READY 时清零基准转速
                    g_sys_context.base_speed = 0;
                    
                    // 打印波形
                    APP_Control_DebugPrint();
                    
                    // 回归就绪空闲
                    g_sys_context.system_step = SYS_STEP_READY;
                    Debug_Printf("[SYS] System State -> READY, Final Halls Archived to Flash.\r\n");
                }
                break;
            }

            default:
                break;
        }
    }
}
