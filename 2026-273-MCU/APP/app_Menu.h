#ifndef __APP_MENU_H
#define __APP_MENU_H

#include "app_main.h"

// ==============================================================================
// 菜单参数调节权限宏开关 (客户锁定模式 vs 研发全开放模式)
// 1: 启用研发全开放模式 (所有参数可调，包括机械固定参数与第3页 PID 参数菜单)
// 0: 客户交付锁定模式 (隐藏丝杆极性/导程/减速比/显示模式，隐藏第3页 PID 菜单，序号自动连续紧凑补齐)
// ==============================================================================
#ifndef MENU_CONFIG_EXPERT_MODE
#define MENU_CONFIG_EXPERT_MODE 0 // 默认 0: 客户交付模式; 改为 1: 研发全开放模式
#endif

#define MENU_CONFIG_SHOW_MECH_PARAMS   MENU_CONFIG_EXPERT_MODE // 机械与显示底层参数开关 (q3/q6/q7/q11)
#define MENU_CONFIG_ENABLE_P3_PID_MENU MENU_CONFIG_EXPERT_MODE // 第3页 PID 参数菜单开关 (-P3-)

// 堵转电流调节 0.3s 自动存 Flash 功能宏开关 (0: 暂时屏蔽/关闭, 1: 开启)
#define ENABLE_STALL_CURRENT_AUTO_SAVE 0

/**
 * @brief 第一页 (-P1-) 菜单项语义枚举
 * @note 当 MENU_CONFIG_SHOW_MECH_PARAMS 为 0 时，固定参数自动剔除，后续项序号自动连续对齐补齐！
 */
typedef enum {
    P1_ITEM_RESET_FACTORY = 0, // 恢复出厂设置开关 (调7生效)
    P1_ITEM_MAX_TRAVEL,        // 升降最大行程上限 (mm)
#if MENU_CONFIG_SHOW_MECH_PARAMS
    P1_ITEM_TARGET_SPEED,      // 整体运行速度 (mm/min)
    P1_ITEM_MOTOR_DIR_INVERT,  // 丝杆旋转方向极性 (0: 正向, 1: 反向)
    P1_ITEM_MAX_SYNC_DIFF,     // 四角最大允许高低误差 (mm)
#endif
    P1_ITEM_STALL_CURRENT,     // 堵转过流保护电流 (0.01A)
#if MENU_CONFIG_SHOW_MECH_PARAMS
    P1_ITEM_LEAD_MM,           // 丝杆导程 (mm)
    P1_ITEM_REDUCTION_RATIO,   // 减速箱减速比
    P1_ITEM_SINGLE_TUNE_STEP,  // 微调移动距离 (mm)
#endif
    P1_ITEM_REBOUND_TRAVEL,    // 下降防夹反弹退回距离 (mm)
    P1_ITEM_COLUMN_MODE,       // 柱体工作模式 (00, 12, 13, 14, 23, 24, 34)
#if MENU_CONFIG_SHOW_MECH_PARAMS
    P1_ITEM_SHOW_CURRENT_MODE, // 数码管显示模式 (0: 仅位置, 1: 仅实时电流, 2: 8项综合)
    P1_ITEM_DRIVER_STALL_PCT,  // 驱动器堵转限流百分比 (%)
#endif
    P1_ITEM_STALL_DETECT_TIME, // 堵转判定防抖时间 (0.1s)
    P1_ITEM_COUNT              // 当前使能的 P1 菜单项总数
} P1_MenuItem_t;

// 二维坐标菜单全局控制变量
extern uint8_t dim1;               // 维度一：0-实时信息监测层 (dim2为电机索引 0~3), 1-常规应用设置层, 2-深层调试只读层, 3-PID控制参数设置层
extern uint8_t dim2;               // 维度二：具体项目编号 / 电机索引
extern uint8_t reset_factory_flag; // 恢复出厂设置开关标记 (0: 否, 1: 恢复)
extern uint16_t adjust_hold_ticks; // 调值不闪烁倒计时 (单位: 50ms 周期)

// 数码管提示动画全局变量
extern char prompt_str[6];    // 数码管提示文本 (如 "-P1-", "-q2-", "-UP-", "-DW-")
extern uint16_t prompt_ticks; // 提示文本保持显示倒计时 (单位: 50ms 周期)

void APP_Menu_SetPrompt(const char *str, uint16_t ticks_50ms);
void APP_MenuTask(void *pvParameters);
uint8_t APP_Menu_GetEditingColumnMode(void);

#endif // __APP_MENU_H
