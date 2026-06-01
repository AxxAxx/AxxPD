#ifndef __BUTTONS_H
#define __BUTTONS_H

#include "stm32g4xx_hal.h"

typedef enum {
    BTN_NONE = 0,
    BTN_INC_PRESS,
    BTN_DEC_PRESS,
    BTN_SEL_PRESS,
    BTN_PWR_SHORT,
    BTN_PWR_LONG,
    BTN_SEL_LONG,
    BTN_INC_REPEAT,
    BTN_DEC_REPEAT,
} ButtonEvent_t;

void Buttons_Init(void);
void Buttons_Tick(void);            /* call every 5ms from SysTick */
void Buttons_Update(void);          /* no-op, kept for API compat */
ButtonEvent_t Buttons_GetEvent(void);

#endif /* __BUTTONS_H */
