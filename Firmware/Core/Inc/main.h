/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* Fault event log */
typedef struct {
    uint32_t tick;
    uint8_t  source;
    uint16_t voltage_mv;
    uint16_t current_ma;
} FaultLog_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Output_Enable(void);
uint8_t Output_Enable_Guarded(void);  /* CLI/remote: 0=OK, 1=toggle cooldown, 2=thermal */
void Output_Disable(void);
void Output_Disable_ISR(void);  /* ISR-safe: GPIO+bleed only, defers flash save */
void OVP_SetThreshold(uint32_t vbus_max_mv);
uint8_t get_hw_version(void);

/* Energy-screen session stats (peaks + runtime since accumulator reset) */
void Energy_SessionReset(void);
extern volatile uint32_t g_session_t0;
extern volatile float g_peak_current_a;
extern volatile float g_peak_power_w;

/* Fault source codes (sequential — use == not &) */
#define FAULT_NONE            0
#define FAULT_INA228_OCP      1
#define FAULT_COMP_OVP        2
#define FAULT_LTC4368         3
#define FAULT_TPD4S480        4
#define FAULT_LM5166_PGOOD    5
#define FAULT_OPP             6
#define FAULT_TIMER           7
#define FAULT_AH_LIMIT        8
#define FAULT_WH_LIMIT        9
#define FAULT_CHARGE_COMPLETE 10
#define FAULT_THERMAL         11

#define FAULT_LOG_SIZE  8

void       FaultLog_Push(uint8_t source, uint16_t mv, uint16_t ma);
FaultLog_t* FaultLog_Get(uint8_t index);
uint8_t    FaultLog_Count(void);

/* Timer auto-shutoff */
uint16_t   Timer_GetRemaining(void);
void       Timer_Start(void);
void       Timer_Stop(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* --- CubeMX-generated pin definitions (G4 board) --- */
#define V_BIT_1_Pin GPIO_PIN_13
#define V_BIT_1_GPIO_Port GPIOC
#define V_BIT_2_Pin GPIO_PIN_14
#define V_BIT_2_GPIO_Port GPIOC
#define V_BIT_3_Pin GPIO_PIN_15
#define V_BIT_3_GPIO_Port GPIOC
#define TPD4S480_FLT_Pin GPIO_PIN_1
#define TPD4S480_FLT_GPIO_Port GPIOF
#define LTC4368_SHDN_Pin GPIO_PIN_1
#define LTC4368_SHDN_GPIO_Port GPIOA
#define LTC4368_FLT_Pin GPIO_PIN_3
#define LTC4368_FLT_GPIO_Port GPIOA
#define TFT_CS_Pin GPIO_PIN_4
#define TFT_CS_GPIO_Port GPIOA
#define TFT_DC_Pin GPIO_PIN_4
#define TFT_DC_GPIO_Port GPIOC
#define TFT_RESET_Pin GPIO_PIN_0
#define TFT_RESET_GPIO_Port GPIOB
#define V_SENSE_Pin GPIO_PIN_1
#define V_SENSE_GPIO_Port GPIOB
#define NTC_Pin GPIO_PIN_2
#define NTC_GPIO_Port GPIOB
#define INA228_ALERT_Pin GPIO_PIN_11
#define INA228_ALERT_GPIO_Port GPIOB
#define SW_1_Pin GPIO_PIN_12
#define SW_1_GPIO_Port GPIOB
#define SW_2_Pin GPIO_PIN_13
#define SW_2_GPIO_Port GPIOB
#define SW_3_Pin GPIO_PIN_14
#define SW_3_GPIO_Port GPIOB
#define SW_4_Pin GPIO_PIN_15
#define SW_4_GPIO_Port GPIOB
#define BLEED_CTRL_Pin GPIO_PIN_6
#define BLEED_CTRL_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOA
#define LM5166_FGOOD_Pin GPIO_PIN_9
#define LM5166_FGOOD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* --- Application aliases for G0→G4 compat --- */
#define VERSION_BIT_1_Pin       V_BIT_1_Pin
#define VERSION_BIT_1_GPIO_Port V_BIT_1_GPIO_Port
#define VERSION_BIT_2_Pin       V_BIT_2_Pin
#define VERSION_BIT_2_GPIO_Port V_BIT_2_GPIO_Port
#define VERSION_BIT_3_Pin       V_BIT_3_Pin
#define VERSION_BIT_3_GPIO_Port V_BIT_3_GPIO_Port

/* Button aliases: SW_1..4 → logical names used by buttons.c / ui.c
 * Physical PCB wiring: SW3(PB14)=left, SW2(PB13)=mid, SW1(PB12)=right-mid, SW4(PB15)=right
 * Logical layout (left to right): DEC | INC | SEL | PWR */
#define BUTTON_DECREASE_Pin       SW_3_Pin
#define BUTTON_DECREASE_GPIO_Port SW_3_GPIO_Port
#define BUTTON_INCREASE_Pin       SW_2_Pin
#define BUTTON_INCREASE_GPIO_Port SW_2_GPIO_Port
#define BUTTON_SELECT_Pin         SW_1_Pin
#define BUTTON_SELECT_GPIO_Port   SW_1_GPIO_Port
#define BUTTON_POWER_Pin          SW_4_Pin
#define BUTTON_POWER_GPIO_Port    SW_4_GPIO_Port

/* LM5166 pin alias (G0 used PGOOD, G4 CubeMX uses FGOOD) */
#define LM5166_PGOOD_Pin          LM5166_FGOOD_Pin
#define LM5166_PGOOD_GPIO_Port    LM5166_FGOOD_GPIO_Port

/* OVP voltage divider ratio (V_SENSE resistor network on G4 board) */
#define OVP_DIVIDER_RATIO (2.2f / (100.0f + 2.2f))

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
