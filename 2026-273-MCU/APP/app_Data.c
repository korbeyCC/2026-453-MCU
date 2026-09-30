#include "app_Data.h"
#include "mid_FLASH.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_Comm.h"
#include <string.h>

APP_DATA_HandleTypeDef app_data;
uint8_t EEPROMSet = 0;

/**
 * @brief  初始化参数系统，读出 Flash 记忆值并进行未初始化检测
 */
void APP_Data_Init(void)
{
    uint32_t addr = MID_FLASH_AddressTransition(0);
    MID_FLASH_ReadData(&addr, sizeof(APP_DATA_HandleTypeDef), (uint16_t *)&app_data);

    // 如果 Flash 尚未初始化
    if (app_data.min_mount_halls[0] == -1 ||
        app_data.min_mount_halls[0] == 0x7FFFFFFF ||
        app_data.max_travel_range_mm <= 0 ||
        app_data.reduction_ratio == 0xFFFF ||
        app_data.reduction_ratio == 0) {

        // 恢复全部出厂默认值并存盘
        APP_Data_ResetDefault();
    }

    // 极性参数合法性防护 (仅允许 0 或 1，若非法或未配置默认置 1 反向丝杆)
    if (app_data.motor_dir_invert > 1) {
        app_data.motor_dir_invert = 1;
    }

    // 堵转电流阈值边界防御 (限制在 [STALL_CURRENT_THRESHOLD_MIN, STALL_CURRENT_THRESHOLD_MAX] 之内)
    if (app_data.stall_current_threshold > STALL_CURRENT_THRESHOLD_MAX) {
        app_data.stall_current_threshold = STALL_CURRENT_THRESHOLD_MAX;
    } else if (app_data.stall_current_threshold < STALL_CURRENT_THRESHOLD_MIN) {
        app_data.stall_current_threshold = STALL_CURRENT_THRESHOLD_MIN;
    }

    // 柱体模式参数合法性防御 (仅允许 0, 12, 13, 14, 23, 24, 34)
    if (app_data.column_mode != 0 && app_data.column_mode != 12 &&
        app_data.column_mode != 13 && app_data.column_mode != 14 &&
        app_data.column_mode != 23 && app_data.column_mode != 24 &&
        app_data.column_mode != 34) {
        app_data.column_mode = 0;
    }
    if (app_data.column_mode_locked > 1) {
        app_data.column_mode_locked = 0;
    }

    // 显示模式参数合法性防御 (0: 仅显示位置, 1: 仅显示实时电流, 2: 8项综合监测模式)
    if (app_data.show_current_mode > 2) {
        app_data.show_current_mode = 0;
    }

    // 驱动器堵转限流百分比防御 (50% ~ 250%，默认 150%)
    if (app_data.driver_stall_percent < 50 || app_data.driver_stall_percent > 250) {
        app_data.driver_stall_percent = 150;
    }

    // 堵转判定防抖时间防御 (100ms ~ 2000ms，默认 600ms)
    if (app_data.stall_detect_time_ms < 100 || app_data.stall_detect_time_ms > 2000) {
        app_data.stall_detect_time_ms = 600;
    }

    // PID 控制参数防御与旧 Flash 兼容自愈 (0xFFFF 异常自愈至默认配置，允许为 0 纯裸跑)
    if (app_data.pid_kp_x1000 > 9999) {
        app_data.pid_kp_x1000 = 300; // 默认 0.300
    }
    if (app_data.pid_ki_x1000 > 5000) {
        app_data.pid_ki_x1000 = 1; // 默认 0.001
    }
    if (app_data.pid_kd_x1000 > 9999) {
        app_data.pid_kd_x1000 = 0; // 默认 0.000
    }
    if (app_data.pid_diff_low_thresh < 10 || app_data.pid_diff_low_thresh > 500) {
        app_data.pid_diff_low_thresh = 50; // 默认 50 counts
    }
    if (app_data.pid_diff_high_thresh < 100 || app_data.pid_diff_high_thresh > 5000) {
        app_data.pid_diff_high_thresh = 1900; // 默认 1900 counts
    }
    if (app_data.pid_out_max_low < 50 || app_data.pid_out_max_low > 1000) {
        app_data.pid_out_max_low = 300; // 默认 300 RPM
    }
    if (app_data.pid_out_max_high < 100 || app_data.pid_out_max_high > 1500) {
        app_data.pid_out_max_high = 700; // 默认 700 RPM
    }
    if (app_data.pid_iout_max_low > 200) {
        app_data.pid_iout_max_low = 20; // 默认 20 RPM
    }
    if (app_data.pid_iout_max_high > 300) {
        app_data.pid_iout_max_high = 40; // 默认 40 RPM
    }
    if (app_data.pid_lag_bias_factor_x100 > 100) {
        app_data.pid_lag_bias_factor_x100 = 20; // 默认 0.20 (偏好落后轴 80%)
    }

    // 强制使能 PVD 检测及硬件 PLS 阈值配置 (PVD 检测阈值调低至 2.6V，防范负载及波动噪声)
    // 注：由于 CubeMX 已经自动生成了 NVIC (PVD_IRQn) 中断使能，此处仅需配置并使能 PVD 硬件外设本身即可
    PWR_PVDTypeDef getConfigPVD;
    getConfigPVD.PVDLevel = PWR_PVDLEVEL_4;
    getConfigPVD.Mode     = PWR_PVD_MODE_IT_RISING; // 电压跌落至阈值以下触发中断
    HAL_PWR_ConfigPVD(&getConfigPVD);
    HAL_PWR_EnablePVD();
}

/**
 * @brief 根据当前 column_mode 返回硬件有效电机掩码
 * @return 8位掩码，第 0~3 位对应电机 1~4
 */
uint8_t App_Data_GetColumnMotorMask(void)
{
    switch (app_data.column_mode) {
        case 12:
            return 0x03; // 轴 1 + 轴 2 (0b0011)
        case 13:
            return 0x05; // 轴 1 + 轴 3 (0b0101)
        case 14:
            return 0x09; // 轴 1 + 轴 4 (0b1001)
        case 23:
            return 0x06; // 轴 2 + 轴 3 (0b0110)
        case 24:
            return 0x0A; // 轴 2 + 轴 4 (0b1010)
        case 34:
            return 0x0C; // 轴 3 + 轴 4 (0b1100)
        case 0:
        default:
            return 0x0F; // 4 柱全使能 (0b1111)
    }
}

/**
 * @brief 恢复出厂默认参数设置并刷新 Flash 固化存盘
 */
void APP_Data_ResetDefault(void)
{
    // ===================================================================
    // 第一页设置菜单参数 (-P1- 常用配置项 q1 ~ q13)
    // ===================================================================
    app_data.max_travel_range_mm     = 1600; // q1:  升降总行程范围 (单位: mm，默认 1000)
    app_data.target_speed_mm_min     = 400;  // q2:  整体运行速度 (单位: mm/min，默认 800)
    app_data.motor_dir_invert        = 1;    // q3:  丝杆运动方向极性 (0: 默认正向, 1: 极性反转)
    app_data.max_sync_diff_mm        = 20;   // q4:  最大同步差阈值 (单位: mm，默认 20)
    app_data.stall_current_threshold = 800;  // q5:  堵转电流阈值 (单位: 0.01A，默认 800 = 8.00A)
    app_data.lead_mm                 = 8;    // q6:  丝杆导程 (单位: mm，默认 8)
    app_data.reduction_ratio         = 30;   // q7:  减速比 (默认 30)
    app_data.single_tune_step_mm     = 2;    // q8:  单轴微调步进距离 (单位: mm，默认 2)
    app_data.rebound_travel_mm       = 500;  // q9:  堵转反弹行程 (单位: mm，默认 500)
    app_data.column_mode             = 0;    // q10: 柱体模式 (0: 4柱00, 12, 13, 14, 23, 24, 34)
    app_data.column_mode_locked      = 0;    //      柱体模式单向锁定状态 (0: 未锁定出厂态自由任选, 1: 已锁定)
    app_data.show_current_mode       = 2;    // q11: 主界面显示模式 (0: 仅位置, 1: 仅实时电流, 2: 8项综合监测)
    app_data.driver_stall_percent    = 150;  // q12: 驱动器堵转限流百分比 (50 ~ 250%, 默认 150%)
    app_data.stall_detect_time_ms    = 1000; // q13: 堵转判定防抖时间 (单位: ms, 默认 1000ms = 1.0s)

    // 非菜单硬件常数
    app_data.hall_coef = 30; // 霍尔传感器每转计数系数 (默认 30)

    // ===================================================================
    // 第三页设置菜单参数 (-P3- PID 控制器配置项 q0 ~ q9)
    // ===================================================================
    app_data.pid_kp_x1000             = 200;   // q0: PID Kp 比例增益 (放大 1000 倍, 300 -> 0.300)
    app_data.pid_ki_x1000             = 0;    // q1: PID Ki 积分增益 (放大 1000 倍, 1 -> 0.001)
    app_data.pid_kd_x1000             = 0;    // q2: PID Kd 微分增益 (放大 1000 倍, 0 -> 0.000)
    app_data.pid_diff_low_thresh      = 50;   // q3: PID 动态限幅低偏差门限 (counts, 默认 50)
    app_data.pid_diff_high_thresh     = 2000; // q4: PID 动态限幅高偏差门限 (counts, 默认 1900)
    app_data.pid_out_max_low          = 1000;  // q5: 小偏差 PID 限幅转速 (RPM, 默认 300)
    app_data.pid_out_max_high         = 1500;  // q6: 大偏差 PID 限幅转速 (RPM, 默认 700)
    app_data.pid_iout_max_low         = 20;   // q7: 小偏差 PID 积分限幅 (RPM, 默认 20)
    app_data.pid_iout_max_high        = 40;   // q8: 大偏差 PID 积分限幅 (RPM, 默认 40)
    app_data.pid_lag_bias_factor_x100 = 0;    // q9: 落后轴偏好权重因子 (放大 100 倍, 20 -> 0.20)

    float c_per_mm              = (float)(app_data.reduction_ratio * app_data.hall_coef) / (float)app_data.lead_mm;
    int32_t default_mount_halls = (int32_t)(1000.0f * c_per_mm + 0.5f);

    for (int i = 0; i < 4; i++) {
        app_data.min_mount_halls[i] = default_mount_halls;
        app_data.motor_abs_halls[i] = default_mount_halls;
    }

    APP_Data_Storage();
}

/**
 * @brief  立即同步将参数保存至 Flash 芯片中 (使用中断安全的 CPU 级别全局中断开关)
 */
void APP_Data_Storage(void)
{
    // 使用 CPU 级别的全局中断开关保护 Flash 擦写，避免在 PVD 中断里调用 taskENTER_CRITICAL 触发 FreeRTOS 断言卡死
    __disable_irq();

    uint32_t base_addr   = MID_FLASH_AddressTransition(0);
    uint32_t target_addr = base_addr;

    MID_FLASH_Unlock();
    MID_FLASH_ErasePage(base_addr);
    MID_FLASH_WrithData(&target_addr, sizeof(APP_DATA_HandleTypeDef), (uint16_t *)&app_data);
    MID_FLASH_Lock();

    __enable_irq();
}

/**
 * @brief  系统设置修改备份守护任务 (EEPROMSet 置 1 时触发，主要用于菜单/用户微调保存)
 */
void APP_Data_Task(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while (1) {
        if (EEPROMSet == 1) {
            APP_Data_Storage();
            EEPROMSet = 0;
        }
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(200));
    }
}

/**
 * @brief  STM32 PWR PVD programmable voltage detector interrupt callback
 * @note   在单片机外部电源掉电瞬间触发，利用电容余电紧急将 4 路电机的绝对霍尔高度持久化归档至 Flash
 */
void HAL_PWR_PVDCallback(void)
{
    static uint8_t s_pvd_locked = 0;
    if (s_pvd_locked) return; // 若已锁死，本次掉电中绝不再执行第二次擦写，防范电容回弹及降压复位

    // 双保险校验一：只有当检测到 PVDO 标志确实为 SET 时，说明 VDD 供电电压确实已低于 2.6V，发生了真实的物理断电
    // 从而 100% 过滤掉上电时电压爬升期的 EXTI 边沿误触发中断
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) != RESET) {
        // 双保险校验二：只有当系统已完成各电机配置初始化且处于正常霍尔轮询状态时，读出的高度数据才有效
        // 这样可以彻底防止上电初期的 0 高度数据意外覆盖 Flash 历史有效绝对高度
        uint8_t sys_ready = (g_sys_context.system_step >= SYS_STEP_READY) ? 1 : 0;

        if (sys_ready) {
            s_pvd_locked = 1; // 紧急上锁保护

            // 1. 紧急归档当前内存中最实时精确的立柱绝对高度
            for (int i = 0; i < 4; i++) {
                app_data.motor_abs_halls[i] = g_sys_context.g_motor_status[i].current_abs_hall;
            }

            // 2. 紧急执行 Flash 存储擦写 (利用约 20ms 电容维持供电期快速写入)
            APP_Data_Storage();
        }
    }
}
