#ifndef __INA228_H
#define __INA228_H

#include "stm32g4xx_hal.h"

#define INA228_ADDR         (0x40 << 1)  /* A0=GND, A1=GND */

/* Register addresses */
#define INA228_REG_CONFIG       0x00
#define INA228_REG_ADC_CONFIG   0x01
#define INA228_REG_SHUNT_CAL    0x02
#define INA228_REG_VSHUNT       0x04
#define INA228_REG_VBUS         0x05
#define INA228_REG_DIETEMP      0x06
#define INA228_REG_CURRENT      0x07
#define INA228_REG_POWER        0x08
#define INA228_REG_ENERGY       0x09
#define INA228_REG_CHARGE       0x0A
#define INA228_REG_DIAG_ALRT    0x0B
#define INA228_REG_SOVL         0x0C
#define INA228_REG_BOVL         0x0E
#define INA228_REG_MFG_ID       0x3E
#define INA228_REG_DEV_ID       0x3F

typedef struct {
    I2C_HandleTypeDef *hi2c;
    float current_lsb;
    float shunt_ohms;
} INA228_t;

typedef struct {
    float voltage_v;
    float current_a;
    float power_w;
    float energy_wh;
    float charge_ah;
    float die_temp_c;
} INA228_Reading_t;

/* Idle callback — called between sequential I2C reads in INA228_ReadAll().
 * Set this to axxpd_run() so the PD stack ticks during the ~9ms read burst. */
void INA228_SetIdleCallback(void (*cb)(void));

HAL_StatusTypeDef INA228_Init(INA228_t *dev, I2C_HandleTypeDef *hi2c, float shunt_ohms, float max_current_a);
HAL_StatusTypeDef INA228_ReadAll(INA228_t *dev, INA228_Reading_t *reading);
HAL_StatusTypeDef INA228_ReadVoltage(INA228_t *dev, float *voltage_v);
HAL_StatusTypeDef INA228_ReadCurrent(INA228_t *dev, float *current_a);
HAL_StatusTypeDef INA228_ResetEnergy(INA228_t *dev);
HAL_StatusTypeDef INA228_SetAlertOverCurrent(INA228_t *dev, float amps);
HAL_StatusTypeDef INA228_ClearAlertLatch(INA228_t *dev);

#endif /* __INA228_H */
