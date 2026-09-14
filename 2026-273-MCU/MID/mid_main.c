#include "mid_main.h"

void MID_Init(void)
{
    BSP_Init();
    MID_TM1650_Init();
    MID_KEY_HardwareBinding();
    MID_SIGNAL_HardwareBinding();
    MID_LED_Init();
    MID_RunLED_Init();
    MID_Buzzer_Init();
    MID_Brake_Init();
    MID_Modbus_Init();
    Sys_Supervisor_Init();
}
