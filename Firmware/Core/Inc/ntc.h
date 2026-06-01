#ifndef __NTC_H
#define __NTC_H

#include "stm32g4xx_hal.h"

void NTC_Init(ADC_HandleTypeDef *hadc);
float NTC_ReadTemperature(void);  /* returns degrees C */

#endif /* __NTC_H */
