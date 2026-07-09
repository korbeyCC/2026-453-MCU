#ifndef __MID_TM1650_H
#define __MID_TM1650_H

// TM1650 引脚宏定义自适应与硬件兜底 (默认SCL->PB15, SDA->PB14)
#ifndef TM1650_SCL_Pin
#define TM1650_SCL_Pin GPIO_PIN_15
#endif
#ifndef TM1650_SCL_GPIO_Port
#define TM1650_SCL_GPIO_Port GPIOB
#endif
#ifndef TM1650_SDA_Pin
#define TM1650_SDA_Pin GPIO_PIN_14
#endif
#ifndef TM1650_SDA_GPIO_Port
#define TM1650_SDA_GPIO_Port GPIOB
#endif

#include "mid_main.h"

void MID_TM1650_Init(void);
void MID_TM1650_DisplayWrite(uint8_t *seg_data, uint8_t len);
uint8_t MID_TM1650_ReadKey(void);

#endif
