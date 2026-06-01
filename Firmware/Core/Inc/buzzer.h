#ifndef __BUZZER_H
#define __BUZZER_H

#include "stm32g4xx_hal.h"

void Buzzer_Init(TIM_HandleTypeDef *htim);
void Buzzer_Beep(uint16_t freq_hz, uint16_t duration_ms);
void Buzzer_Update(void);  /* call from main loop */
void Buzzer_Off(void);

/* Named beep presets — all use a consistent base frequency (800 Hz). */
void Buzzer_Click(void);       /* Button press: short tick, 8ms          */
void Buzzer_Confirm(void);     /* Selection confirm: 20ms                */
void Buzzer_Fault(void);       /* HW fault / thermal: loud 320ms         */
void Buzzer_Enable(void);      /* Output enable: high chirp 1200Hz 60ms  */
void Buzzer_Disable(void);     /* Output disable / emergency off: 120ms  */
void Buzzer_Warn(void);        /* Temperature warning: short chirp 30ms  */
void Buzzer_FreqSweep(void);   /* Test sweep 800-5000Hz to find resonance */

#endif /* __BUZZER_H */
