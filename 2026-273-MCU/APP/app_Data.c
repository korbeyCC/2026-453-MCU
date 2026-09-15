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
        case 12: return 0x03; // 轴 1 + 轴 2 (0b0011)
        case 13: return 0x05; // 轴 1 + 轴 3 (0b0101)
        case 14: return 0x09; // 轴 1 + 轴 4 (0b1001)
        case 23: return 0x06; // 轴 2 + 轴 3 (0b0110)
        case 24: return 0x0A; // 轴 2 + 轴 4 (0b1010)
        case 34: return 0x0C; // 轴 3 + 轴 4 (0b1100)
        case 0:
        default: return 0x0F; // 4 柱全使能 (0b1111)
    }
}

/**
 * @brief 恢复出厂默认参数设置并刷新 Flash 固化存盘
 */
void APP_Data_ResetDefault(void)
{
    app_data.max_travel_range_mm     = 2000; // 默认 2000 mm
    app_data.reduction_ratio         = 30;   // 默认减速比 30
    app_data.target_speed_mm_min     = 750;  // 默认 750 mm/min
    app_data.stall_current_threshold = 700;  // 默认 7.00A (需低于驱动器堵转上限 STALL_CURRENT_THRESHOLD_MAX = 7.5A*120% = 9.00A = 900)
    app_data.max_sync_diff_mm        = 5;    // 默认 5 mm
    app_data.lead_mm                 = 8;    // 默认 8 mm
    app_data.hall_coef               = 30;   // 默认 30
    app_data.motor_dir_invert        = 1;    // 默认 1 (反向丝杆，极性反转)
    app_data.single_tune_step_mm     = 1;    // 默认微调步进 1 mm
    app_data.rebound_travel_mm       = 1000; // 默认堵转反弹行程 1000 mm (1米)
    app_data.column_mode             = 0;    // 默认四柱模式 (00)
    app_data.column_mode_locked      = 0;    // 默认未锁定 (出厂态自由任选)

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
