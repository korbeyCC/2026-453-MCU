#ifndef __MID_LED_H
#define __MID_LED_H

#include "mid_main.h"
#include <stdbool.h>

typedef enum {
    MID_LED_1 = 0,
    MID_LED_2,
    MID_LED_COUNT
} MID_LED_ID;

void MID_LED_Init(void);
void MID_LED_Write(MID_LED_ID id, bool state);
void MID_LED_Toggle(MID_LED_ID id);

#endif
