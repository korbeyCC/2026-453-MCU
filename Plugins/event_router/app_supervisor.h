#ifndef __APP_SUPERVISOR_H
#define __APP_SUPERVISOR_H

#include "mid_router.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==============================================================================
// 1. 系统顶级运行模式 (System Operation Mode) —— 单一可信源 (SSOT)
// ==============================================================================
typedef enum {
    SYS_MODE_STANDBY = 0,     // 待机就绪态 (主界面, 4轴读数监测, 接受运动与微调, 允许进菜单)
    SYS_MODE_MOTION,          // 闭环运行态 (正在升降/微调中, 锁定菜单入口)
    SYS_MODE_AUTO_ALIGN,      // 自动调平态 (专用算法闭环, 屏蔽常规升降输入)
    SYS_MODE_MENU_CONFIG,     // 菜单调参态 (按键独占K1~K6, 电机强制闭锁停机, 防误动)
    SYS_MODE_DEBUG_CALIB,     // 深层调试态 (纯只读安装零点mm)
    SYS_MODE_FAULT_LOCKED     // 故障锁定态 (报警闪烁落盘, 仅响应消警与复位)
} System_Mode_t;

// ==============================================================================
// 2. 指示灯与蜂鸣器联动状态定义 (供 app_show 纯读使用)
// ==============================================================================
typedef enum {
    SYS_IND_FAULT = 0,        // 故障急停双闪 (1Hz/2Hz 警示闪烁 + 报警鸣叫)
    SYS_IND_RUNNING,          // 运行中指示 (工作指示灯连续闪烁 + 运动蜂鸣)
    SYS_IND_STEADY            // 待机就绪态 (指示灯常亮 + 静音)
} Sys_Indicator_State_t;

// ==============================================================================
// 3. 核心决策中枢 API 与系统状态机门禁
// ==============================================================================

/**
 * @brief  初始化业务决策中枢并挂载策略过滤器至路由器引擎
 */
void APP_Supervisor_Init(void);

// 兼容别名
#define Sys_Supervisor_Init APP_Supervisor_Init

/**
 * @brief  切换系统全局顶级主模式
 */
void Sys_Mode_Set(System_Mode_t new_mode);

/**
 * @brief  获取当前系统全局顶级主模式
 */
System_Mode_t Sys_Mode_Get(void);

/**
 * @brief  权威门禁：当前是否允许启动电机运动
 */
bool Sys_Mode_CanRunMotion(void);

/**
 * @brief  权威门禁：当前是否允许进入参数设置菜单
 */
bool Sys_Mode_CanEnterMenu(void);

/**
 * @brief  查询当前菜单是否正处于调参活跃状态 (独占按键)
 */
bool Sys_Mode_IsMenuActive(void);

/**
 * @brief  查询当前是否处于严重故障锁定状态
 */
bool Sys_Mode_IsFaultLocked(void);

// ==============================================================================
// 4. 系统只读视图模型接口 (System View Model / Telemetry) —— 供 app_show / app_Menu 纯读
// ==============================================================================

int32_t Sys_View_GetAxisTravelMm(uint8_t axis_idx);
int32_t Sys_View_GetAxisMountMm(uint8_t axis_idx);
uint8_t Sys_View_GetFaultCode(void);
bool Sys_View_IsRebounding(void);
Sys_Indicator_State_t Sys_View_GetIndicatorState(void);

void Sys_View_SetTuneDir(uint8_t dir);
uint8_t Sys_View_GetTuneDir(void);

// ==============================================================================
// 5. 中立通知与请求接口 (Decoupled Notification / Command Sinks)
// ==============================================================================

void Sys_Notify_ParamsUpdated(void);
void Sys_Notify_FactoryReset(void);
bool Sys_Request_ClearFault(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SUPERVISOR_H */
