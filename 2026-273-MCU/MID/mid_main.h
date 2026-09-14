#ifndef __MID_MAIN_H
#define __MID_MAIN_H

#include "main.h"
#include "cmsis_os.h"
#include "bsp_main.h"

#include "mid_SEG.h"
#include "mid_TM1650.h"
#include "mid_Key.h"
#include "mid_FLASH.h"
#include "mid_signal.h"
#include "mid_led.h"
#include "mid_modbus.h"
#include "mid_run_led.h"
#include "mid_buzzer.h"
#include "mid_brake.h"
#include "mid_supervisor.h"

void MID_Init(void);

#endif
