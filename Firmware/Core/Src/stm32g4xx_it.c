/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "buttons.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern void axxpd_tick_pd(void);
extern void axxpd_ucpd_irq(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* [FIX 2026-08-01] Fault-safe shutdown: on any CPU fault, force the output
 * OFF before anything else so a crash can never leave up to 48V/5A live on
 * the terminals. Direct register writes only — HAL must not be trusted from
 * a fault context. BSRR/BRR writes are atomic and always safe.
 * SHDN (PA1) LOW  = LTC4368 output disabled.
 * BLEED_CTRL (PC6) HIGH = discharge the output capacitance. */
static inline void fault_kill_output(void)
{
  LTC4368_SHDN_GPIO_Port->BRR  = LTC4368_SHDN_Pin;   /* SHDN low: output OFF */
  BLEED_CTRL_GPIO_Port->BSRR   = BLEED_CTRL_Pin;     /* bleed on: discharge  */
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /* [FIX 2026-08-01] Output off first — never leave the terminals live. */
  fault_kill_output();

  /* [FIX 2026-08-01] Flash double-bit ECC error raises NMI. Clear the ECC
   * flags (rc_w1) so the NMI does not immediately re-fire, then take a clean
   * reset instead of hanging until the IWDG bites and the fault loops. */
  if (FLASH->ECCR & (FLASH_ECCR_ECCD | FLASH_ECCR_ECCC))
  {
    FLASH->ECCR = FLASH_ECCR_ECCD | FLASH_ECCR_ECCC;
  }

  NVIC_SystemReset();
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /* [FIX 2026-08-01] Output off first, then clean reset (IWDG took ~5s). */
  fault_kill_output();
  NVIC_SystemReset();
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  /* [FIX 2026-08-01] Output off first, then clean reset. */
  fault_kill_output();
  NVIC_SystemReset();
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  /* [FIX 2026-08-01] Output off first, then clean reset. */
  fault_kill_output();
  NVIC_SystemReset();
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  /* [FIX 2026-08-01] Output off first, then clean reset. */
  fault_kill_output();
  NVIC_SystemReset();
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  static uint8_t pd_div = 0;
  static uint8_t btn_div = 0;

  /* PD stack safety-net tick every 2ms */
  if (++pd_div >= 2) {
      pd_div = 0;
      axxpd_tick_pd();
  }

  /* Button debounce tick every 5ms */
  if (++btn_div >= 5) {
      btn_div = 0;
      Buttons_Tick();
  }
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  /* UCPD RX DMA — completion handled by UCPD RXMSGEND flag */
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
  /* UCPD TX DMA — completion handled by UCPD TXMSGSENT flag */
  /* USER CODE END DMA1_Channel2_IRQn 0 */
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt (SPI1 TX for LCD).
  */
void DMA1_Channel3_IRQHandler(void)
{
  extern DMA_HandleTypeDef hdma_spi1_tx;
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  extern UART_HandleTypeDef huart2;
  HAL_UART_IRQHandler(&huart2);
}

/**
  * @brief This function handles COMP1/2/3 interrupt via EXTI21/22/29.
  */
void COMP1_2_3_IRQHandler(void)
{
  extern COMP_HandleTypeDef hcomp1;
  HAL_COMP_IRQHandler(&hcomp1);
}

/**
  * @brief This function handles USB low priority interrupt remap.
  */
void USB_LP_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_IRQn 0 */

  /* USER CODE END USB_LP_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_IRQn 1 */

  /* USER CODE END USB_LP_IRQn 1 */
}

/**
  * @brief This function handles UCPD1 interrupt.
  */
void UCPD1_IRQHandler(void)
{
  /* USER CODE BEGIN UCPD1_IRQn 0 */
  axxpd_ucpd_irq();
  return;  /* pdsink handles everything — skip CubeMX USBPD handler */
  /* USER CODE END UCPD1_IRQn 0 */

  /* USER CODE BEGIN UCPD1_IRQn 1 */

  /* USER CODE END UCPD1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief EXTI line 1 — TPD4S480 ESD fault (PF1)
  */
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(TPD4S480_FLT_Pin);
}

/**
  * @brief EXTI line 3 — LTC4368 fault (PA3)
  */
void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(LTC4368_FLT_Pin);
}

/**
  * @brief EXTI lines 5-9 — LM5166 power-good (PB9)
  */
void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(LM5166_FGOOD_Pin);
}

/**
  * @brief EXTI lines 10-15 — INA228 alert (PB11), buttons (PB12-15)
  */
void EXTI15_10_IRQHandler(void)
{
  if (__HAL_GPIO_EXTI_GET_IT(INA228_ALERT_Pin))
    HAL_GPIO_EXTI_IRQHandler(INA228_ALERT_Pin);
  if (__HAL_GPIO_EXTI_GET_IT(SW_1_Pin))
    HAL_GPIO_EXTI_IRQHandler(SW_1_Pin);
  if (__HAL_GPIO_EXTI_GET_IT(SW_2_Pin))
    HAL_GPIO_EXTI_IRQHandler(SW_2_Pin);
  if (__HAL_GPIO_EXTI_GET_IT(SW_3_Pin))
    HAL_GPIO_EXTI_IRQHandler(SW_3_Pin);
  if (__HAL_GPIO_EXTI_GET_IT(SW_4_Pin))
    HAL_GPIO_EXTI_IRQHandler(SW_4_Pin);
}

/* USER CODE END 1 */
