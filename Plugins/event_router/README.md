# event_router 事件路由与系统决策分发插件

## 1. 插件概述
`event_router` 是专为单片机多任务协作系统设计的事件路由器与模式决策中枢插件。
采用“机制与策略分离”架构：
- **机制层 (`mid_router.h` / `mid_router.c`)**：纯通用责任链遍历引擎 + 多载体（单值覆盖邮箱 / 任务通知）投递容器，100% 零业务依赖；
- **策略层 (`app_supervisor.h` / `app_supervisor.c`)**：业务模式状态机（SSOT）、4 级业务拦截过滤规则与视图模型（View Model）。

## 2. 目录结构
```text
Plugins/event_router/
├── mid_router.h      # 机制层：事件定义、责任链引擎接口、多载体接口
├── mid_router.c      # 机制层：零业务耦合执行引擎、安全邮箱与通知实现
├── app_supervisor.h  # 策略层：模式定义、视图模型、中立通知与决策中枢 API
├── app_supervisor.c  # 策略层：4 级业务过滤策略实现与行程遥测换算
└── README.md         # 插件说明文档
```

## 3. 快速接入指南

### 3.1 初始化
在 `main.c` 或系统启动任务（如 `defaultTask`）中调用：
```c
#include "app_supervisor.h"

// 启动路由器引擎并自动完成业务策略表挂载
APP_Supervisor_Init();
```

### 3.2 前端事件源上报（如按键、遥控、串口）
驱动层只需包含 `mid_router.h`，将外设动作封装为标准事件后直调分发：
```c
#include "mid_router.h"

Sys_Event_t evt;
evt.source     = SYS_EVT_SRC_KEY;
evt.id         = key_id;
evt.event_type = MID_KEY_EVT_LEASS;
evt.count      = 0;
evt.param      = 0;

if (Sys_Router_Dispatch(&evt) == EVENT_CONSUMED) {
    return; // 已被优先级管道拦截消费
}
// 兜底流入本地队列...
```

### 3.3 后端任务消费

#### 模式 A：控制任务通过单值覆盖邮箱消费（防积压）
```c
#include "mid_router.h"

uint8_t src = 0, id = 0, evt = 0;
if (Sys_Mailbox_GetMotionCmd(&src, &id, &evt)) {
    // 处理最新运动/微调/打断指令...
}
```

#### 模式 B：菜单任务通过任务通知无损消费（0 队列 RAM 开销）
```c
#include "mid_router.h"

// 初始化时注册任务句柄
Sys_Router_RegisterMenuTask(xTaskGetCurrentTaskHandle());

// 循环中等待按键通知
uint8_t k_id = 0, k_evt = 0, k_cnt = 0;
if (Sys_Router_WaitMenuKey(&k_id, &k_evt, &k_cnt, 0)) {
    // 响应按键交互...
}
```

#### 模式 C：显示任务读取中立只读视图模型
```c
#include "app_supervisor.h"

int32_t height_mm = Sys_View_GetAxisTravelMm(axis_idx);
Sys_Indicator_State_t state = Sys_View_GetIndicatorState();
```
