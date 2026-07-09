#ifndef __BSP_MAIN_H
#define __BSP_MAIN_H

#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"

void BSP_Init(void);
void Beep_Ctrl(uint8_t state);

#endif
