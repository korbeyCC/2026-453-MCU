#ifndef __MID_BRAKE_H
#define __MID_BRAKE_H

#include "mid_main.h"
#include <stdbool.h>

/**
 * @brief 2026-453 机械抱闸控制 (PA15 / CF3, 继电器 J3, 输出端子 P7) 驱动接口
 */
void MID_Brake_Init(void);
void MID_Brake_Release(void);
void MID_Brake_Lock(void);
bool MID_Brake_IsReleased(void);

#endif /* __MID_BRAKE_H */
