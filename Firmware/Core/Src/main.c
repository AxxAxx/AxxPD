/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "axxpd_main.h"
#include "ui.h"
#include "settings.h"
#include "ina228.h"
#include "buttons.h"
#include "buzzer.h"
#include "ntc.h"
#include "graph.h"
#include "boot_selector.h"
#include "uart.h"
#include "lcd.h"
#include "splash_logo.h"
#include "stm32g4xx_ll_ucpd.h"
#include "usbd_cdc_if.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

COMP_HandleTypeDef hcomp1;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* --- Fault & output state --- */
volatile uint8_t  g_output_enabled = 0;
volatile uint8_t  g_hw_fault = 0;
volatile uint8_t  g_fault_source = FAULT_NONE;
volatile uint8_t  g_fault_pending_beep = 0;
volatile uint32_t g_fault_suppress_until = 0;
volatile uint8_t  g_usb_initialized = 0;

/* --- OCP retry state machine --- */
static volatile uint8_t  ocp_retry_count   = 0;    /* retries attempted so far */
static volatile uint32_t ocp_retry_tick    = 0;    /* tick when last OCP trip occurred */
static volatile uint8_t  ocp_retry_pending = 0;    /* 1 = waiting to re-enable output */

/* --- Output toggle cooldown (MOSFET thermal protection) --- */
static uint32_t g_output_enable_tick = 0; /* tick of last Output_Enable() call */
#define OUTPUT_TOGGLE_COOLDOWN_MS 1500U   /* min interval between enable events */
static uint8_t    g_boot_pdo = 0;

/* "Power on at boot" — one-shot. Latched from settings at init; the main loop
 * arms the output once the PD contract (incl. EPR) has settled, then clears it.
 * Independent of "remember boot" (which only restores last V/I, no enable). */
static uint8_t    g_boot_power_on = 0;

/* "Restore last V/I" — one-shot. When set, the main loop requests the saved
 * voltage/current once the PD contract (incl. EPR negotiation) has settled, so
 * a saved EPR voltage (e.g. 48V) is applied only after EPR is up. */
static uint8_t    g_boot_restore_vi = 0;

/* Deferred EPR entry — non-blocking, runs in the main loop.
 * Phase 0: waiting for 1.5 s from boot before attempting.
 * Phase 1: EPR requested, waiting for completion (up to 2 s).
 * Phase 2: done (success or gave up). */
static uint8_t    g_epr_boot_phase   = 0;
static uint8_t    g_epr_boot_attempt = 0;
static uint32_t   g_epr_boot_t0      = 0;

/* Autostart (power_on_boot) abort countdown */
static volatile uint8_t g_boot_autostart_aborted = 0;  /* SELECT pressed during the window */
static uint32_t         g_boot_t0 = 0;                  /* HAL_GetTick at boot; countdown reference */

/* Fault event log (ring buffer) */
volatile FaultLog_t g_fault_log[FAULT_LOG_SIZE];
volatile uint8_t g_fault_log_head = 0;
volatile uint8_t g_fault_log_count = 0;

/* INA228 device handle + last reading */
INA228_t g_ina;
INA228_Reading_t g_ina_reading = {0};

/* Energy-screen session stats: tick of last accumulator reset + peaks
 * since then. Reset together with the INA228 charge/energy accumulators
 * via Energy_SessionReset(). */
volatile uint32_t g_session_t0 = 0;
volatile float g_peak_current_a = 0.0f;
volatile float g_peak_power_w   = 0.0f;

void Energy_SessionReset(void)
{
    g_session_t0     = HAL_GetTick();
    g_peak_current_a = 0.0f;
    g_peak_power_w   = 0.0f;
}

/* NTC temperature (updated at 2 Hz) */
volatile float g_ntc_temp = 25.0f;

/* Timer auto-shutoff state */
static volatile uint32_t timer_end_tick = 0;
static volatile uint8_t  timer_running = 0;

/* Data streaming (default 20 Hz, triggered by CLI 'stream on') */
extern volatile uint8_t  g_stream_enabled;      /* defined in ucpd_driver.cpp */
extern volatile uint32_t g_stream_interval_ms;   /* defined in ucpd_driver.cpp */
static uint32_t last_stream_ms = 0;

/* Charge-complete current-below-threshold tracker (file scope so Output_Enable can reset it) */
static uint32_t cc_low_since = 0;

/* Deferred settings save flag — set by Output_Disable_ISR(), consumed by main loop */
static volatile uint8_t g_save_on_disable = 0;

/* Timing cadence trackers */
static uint32_t last_ina_ms = 0;
static uint32_t last_ui_ms = 0;
static uint32_t last_ntc_ms = 0;
static uint32_t last_graph_sample_ms = 0;

/* INA228 sampling period */
#define INA228_SAMPLE_PERIOD_MS   10
/* UI refresh period (~30 Hz) */
#define UI_REFRESH_PERIOD_MS      33
/* NTC temperature poll period */
#define NTC_POLL_PERIOD_MS        500

/* Autostart (power_on_boot): visible abort-countdown window before arming.
 * Output arms at max(negotiation_ready, this window) — overlaps the PD/EPR
 * settle wait so it adds no time beyond the intentional abort window. */
#define AUTOSTART_COUNTDOWN_S     5U

/* Thermal protection thresholds (Celsius) */
#define THERMAL_WARN_C            60
#define THERMAL_SHUTDOWN_C        85
#define THERMAL_COOLDOWN_C        75

/* INA228 / I2C3 health + bus-recovery state.
 * The I2C bus to the INA228 can wedge — a slave holding SDA low, or the
 * peripheral latching its BUSY flag — after which every read fails and the
 * last reading would otherwise stay frozen on screen forever. (Confirmed via
 * SWD: SCL+SDA stuck low, I2C3 ISR.BUSY set, voltage frozen at last value.)
 * We detect repeated read failures, mark the reading untrusted, and recover
 * the bus (bit-bang SCL to free a stuck slave + peripheral reset). */
#define INA_FAIL_RECOVER_N        3U      /* consecutive read failures before recovery */
static uint8_t           g_ina_valid = 0;          /* last reading fresh & trusted */
static uint16_t          g_ina_fail  = 0;          /* consecutive read failures     */
static volatile uint32_t g_ina_recover_count = 0;  /* total bus recoveries (diag)   */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UCPD1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_COMP1_Init(void);
static void MX_TIM8_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */
static void I2C3_BusRecover(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ~5 us busy-wait for the I2C bus-recovery bit-bang. Exact timing is not
 * critical; anything in the 1-10 us range frees a stuck slave. */
static void i2c_recover_delay(void)
{
    for (volatile uint32_t c = 0U; c < (SystemCoreClock / 200000U); c++) { __NOP(); }
}

/* Recover a wedged I2C3 bus (INA228 @ PA8=SCL / PC11=SDA). Bit-bangs up to 9
 * SCL pulses to flush a slave holding SDA low, issues a STOP, then hard-resets
 * and re-inits the peripheral to clear a latched BUSY flag. Does NOT touch the
 * INA228 config, so the energy/charge accumulators are preserved. */
static void I2C3_BusRecover(void)
{
    GPIO_InitTypeDef gp = {0};

    __HAL_I2C_DISABLE(&hi2c3);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gp.Mode  = GPIO_MODE_OUTPUT_OD;
    gp.Pull  = GPIO_PULLUP;
    gp.Speed = GPIO_SPEED_FREQ_LOW;
    gp.Pin   = GPIO_PIN_8;   HAL_GPIO_Init(GPIOA, &gp);   /* SCL = PA8  */
    gp.Pin   = GPIO_PIN_11;  HAL_GPIO_Init(GPIOC, &gp);   /* SDA = PC11 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
    i2c_recover_delay();

    /* Clock SCL until the slave releases SDA (max 9 pulses = one byte + ACK). */
    for (uint8_t i = 0U; i < 9U; i++) {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_SET) break;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        i2c_recover_delay();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        i2c_recover_delay();
    }

    /* STOP condition: SDA low while SCL high, then SDA released high. */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
    i2c_recover_delay();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,  GPIO_PIN_SET);
    i2c_recover_delay();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
    i2c_recover_delay();

    /* Clear the latched BUSY flag and restore normal AF operation. */
    __HAL_RCC_I2C3_FORCE_RESET();
    __HAL_RCC_I2C3_RELEASE_RESET();
    MX_I2C3_Init();                 /* HAL_I2C_Init -> MspInit restores PA8/PC11 AF */
}

/* --- Autostart countdown (power_on_boot) accessors, used by the UI --- */

/* 1 while the boot auto power-on is still pending (counting down / waiting for
 * the contract), 0 once it has armed, been aborted, or was never enabled. */
uint8_t Boot_AutostartPending(void)
{
    return (g_boot_power_on && !g_output_enabled && !g_boot_autostart_aborted) ? 1U : 0U;
}

/* Whole seconds left in the countdown (ceil). 0 means the count is done and we
 * are in the post-countdown "waiting for the PD/EPR contract" state. */
int Boot_AutostartSecsRemaining(void)
{
    uint32_t total   = AUTOSTART_COUNTDOWN_S * 1000U;
    uint32_t elapsed = (uint32_t)(HAL_GetTick() - g_boot_t0);
    if (elapsed >= total) return 0;
    return (int)((total - elapsed + 999U) / 1000U);
}

/* Cancel autostart for this boot only (output never arms; setting unchanged). */
void Boot_AutostartAbort(void)
{
    g_boot_autostart_aborted = 1;
    g_boot_power_on = 0;
}

/* Returns 1 if the charger currently OFFERS the saved voltage — a matching
 * Fixed PDO (±250 mV) or a voltage within a PPS/AVS range — scanning both the
 * SPR and EPR source-cap lists. EPR fixed PDOs (e.g. 48V) only appear here once
 * EPR mode is active and the source has re-advertised its caps, so this is
 * polled until it becomes true (or we time out). */
static uint8_t boot_restore_supported(void)
{
    uint32_t lv = Settings_GetLastVoltage();
    uint32_t pdos[14];
    for (int pass = 0; pass < 2; pass++) {
        uint8_t n = (pass == 0) ? axxpd_get_src_pdos(pdos, 14)
                                : axxpd_get_epr_src_pdos(pdos, 14);
        for (uint8_t i = 0U; i < n; i++) {
            uint32_t pdo = pdos[i];
            if (pdo == 0U) continue;
            uint32_t type = (pdo >> 30) & 0x3U;
            uint32_t sub  = (pdo >> 28) & 0x3U;
            if (type == 0U) {                                   /* Fixed */
                uint32_t mv = ((pdo >> 10) & 0x3FFU) * 50U;
                if (lv + 250U >= mv && lv <= mv + 250U) return 1U;
            } else if (type == 3U && sub == 0U) {               /* PPS APDO */
                uint32_t lo = ((pdo >> 8) & 0xFFU) * 100U;
                uint32_t hi = ((pdo >> 17) & 0xFFU) * 100U;
                if (lv >= lo && lv <= hi) return 1U;
            } else if (type == 3U && sub == 1U) {               /* AVS APDO */
                uint32_t lo = ((pdo >> 8) & 0xFFU) * 100U;
                uint32_t hi = ((pdo >> 17) & 0x1FFU) * 100U;
                if (lo < 15000U) lo = 15000U;
                if (lv >= lo && lv <= hi) return 1U;
            }
        }
    }
    return 0U;
}

/* Request the LOWEST fixed PDO (per USB-PD always at least 5V vSafe5V). Used as
 * the fallback when the charger doesn't offer the saved voltage. */
static void boot_restore_lowest(void)
{
    uint32_t pdos[14];
    uint8_t  n = axxpd_get_src_pdos(pdos, 14);
    uint32_t lowest_mv = 0U, lowest_ma = 0U;
    for (uint8_t i = 0U; i < n; i++) {
        uint32_t pdo = pdos[i];
        if (pdo == 0U) continue;
        if (((pdo >> 30) & 0x3U) == 0U) {                       /* Fixed */
            uint32_t mv = ((pdo >> 10) & 0x3FFU) * 50U;
            uint32_t ma = ((pdo >>  0) & 0x3FFU) * 10U;
            if (lowest_mv == 0U || mv < lowest_mv) { lowest_mv = mv; lowest_ma = ma; }
        }
    }
    if (lowest_mv >= 3300U) axxpd_request_voltage(lowest_mv, lowest_ma);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* === PHASE 1: Boot at 16MHz HSI — negotiate PD contract ===
   * LM5166 soft-start inrush (~825mA peak) can trip charger pre-contract
   * current limit. Boot at 16MHz to minimize MCU load during this window. */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UCPD1_Init();

  axxpd_init();
  {
      uint32_t t0 = HAL_GetTick();
      while ((HAL_GetTick() - t0) < 1000) {
          axxpd_run();
          if (axxpd_get_active_pdo_index() > 0 && (HAL_GetTick() - t0) >= 150)
              break;
      }
  }

  /* === PHASE 2: Switch to 128MHz (APB1=16MHz keeps UCPD timing correct) === */
  SystemClock_Config();
  HAL_PWREx_DisableUCPDDeadBattery();

  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_TIM15_Init();
  MX_USART2_UART_Init();
  MX_USB_Device_Init();
  MX_COMP1_Init();
  MX_TIM8_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */

  /* --- Settings (load before OVP/OCP so values are available) --- */
  INA228_Init(&g_ina, &hi2c3, 0.0068f, 6.0f);  /* 6.8 mΩ shunt (G4 board) */
  Settings_Init();

  g_usb_initialized = 1;

  /* --- OVP hardware setup (use saved threshold, default 55V) --- */
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  {
      uint16_t ovp_mv = Settings_GetOvpMv();
      OVP_SetThreshold(ovp_mv > 0 ? ovp_mv : 55000);
  }
  HAL_COMP_Start(&hcomp1);

  /* --- OCP hardware setup (apply saved INA228 alert threshold) --- */
  /* Arm the ALERT-pin backup OCP layer: INA228_Init's soft reset left the
   * SOVL register at the chip default, so program the saved threshold now.
   * Same call pattern as the settings menu and CLI 'protect ocp'. */
  {
      uint16_t ocp_ma = Settings_GetOcpMa();
      INA228_SetAlertOverCurrent(&g_ina, (float)(ocp_ma > 0 ? ocp_ma : 5500) / 1000.0f);
  }

  /* --- Display splash (gated by Splash Screen setting) --- */
  LCD_init();
  LCD_Fill(0, 0, 319, 479, 0x0000);
  if (Settings_GetSplashScreen()) {
      LCD_DrawImage(97, 26, (UG_BMP*)&splash_logo);
  }

  /* PD stack already initialized in Phase 1+2 above */
  axxpd_enable_tick();
  INA228_SetIdleCallback(axxpd_run);

  /* --- Peripheral init --- */
  Buttons_Init();
  Buzzer_Init(&htim8);
  NTC_Init(&hadc2);
  UART_Init(&huart2);

  /* --- Startup beep (gated by setting) --- */
  if (Settings_GetStartupBeep() && Settings_GetBuzzerEnabled()) {
      Buzzer_Confirm();
  }

  /* Enable cable_emu early so it's ready for SOP' discovery before
   * boot selector triggers EPR entry at the 1-second mark. */
  axxpd_enable_cable_emu();

  /* --- Boot selector + UI --- */
  /* "Restore last V/I" takes precedence: skip the PDO selector screen entirely
   * and go straight to the last used V/I (restored in the main loop once the
   * contract incl. EPR settles). Otherwise run the normal boot PDO selector. */
  if (!Settings_GetRememberBoot()) {
      int sel = BootSelector_Run();
      if (sel > 0) g_boot_pdo = (uint8_t)sel;
  }

  /* Don't clear screen here — boot selector already shows "Requesting X.XV..."
   * message. UI_Init() will draw the dashboard when ready. */

  /* Stabilization: let PD stack settle, then repair the contract flag.
   * HAS_EXPLICIT_CONTRACT can be cleared by hard resets during boot;
   * without it, EPR entry and PDO switching are silently blocked. */
  {
      uint32_t t0 = HAL_GetTick();
      while ((HAL_GetTick() - t0) < 500) {
          IWDG->KR = 0xAAAAU;
          axxpd_run();
          if (axxpd_get_active_pdo_index() > 0) break;
      }
      axxpd_ensure_contract_flag();
  }

  /* EPR entry is handled as a non-blocking phase in the main loop
   * (see g_epr_boot_phase below).  No blocking wait here. */
  if (!axxpd_is_epr_active()) {
      axxpd_disable_epr_intent();
  }

  /* "Restore last V/I" is applied in the main loop (g_boot_restore_vi) AFTER the
   * contract incl. EPR has settled — so a saved EPR voltage is requested only
   * once EPR is up. It does NOT enable the output (that's "Power on boot"). */
  g_boot_restore_vi = (Settings_GetRememberBoot() && g_boot_pdo == 0) ? 1U : 0U;

  /* Latch the "Power on at boot" one-shot.  The main loop arms the output
   * once the PD contract (incl. EPR) has settled — see g_boot_power_on below. */
  g_boot_power_on = Settings_GetPowerOnBoot();

  UI_Init();

  /* "Start locked": boot into locked mode — all buttons rejected except a
   * SELECT hold (SEL_LONG), which unlocks. */
  if (Settings_GetStartLocked()) {
      UI_SetLocked(1);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Start IWDG here — after boot selector and all init is complete.
   * Starting earlier causes watchdog reset during boot selector wait. */
  IWDG->KR  = 0x5555U;   /* unlock */
  IWDG->PR  = 4U;         /* /64 prescaler */
  IWDG->RLR = 2500U;      /* 2500 * (64/32000) = 5.0 s */
  IWDG->KR  = 0xCCCCU;   /* start */

  uint32_t boot_tick = HAL_GetTick(); /* overflow-safe reference for EPR boot */
  g_boot_t0 = boot_tick;              /* shared reference for the autostart countdown */

  while (1)
  {
      uint32_t now = HAL_GetTick();

      IWDG->KR = 0xAAAAU;  /* refresh watchdog */

      /* Boot selector PDO request (one-shot, first iteration only) */
      if (g_boot_pdo > 0) {
          uint32_t pdo = BootSelector_GetSelectedPdo();
          if (pdo != 0U) {
              uint32_t type = (pdo >> 30U) & 0x3U;
              if (type == 0U) {
                  uint32_t mv = ((pdo >> 10U) & 0x3FFU) * 50U;
                  uint32_t ma = ((pdo >>  0U) & 0x3FFU) * 10U;
                  axxpd_request_voltage(mv, ma);
              } else if (type == 3U) {
                  uint32_t sub = (pdo >> 28U) & 0x3U;
                  if (sub == 0U) {
                      uint32_t max_mv = ((pdo >> 17U) & 0xFFU) * 100U;
                      uint32_t max_ma = ((pdo >> 0U) & 0x7FU) * 50U;
                      axxpd_request_voltage(max_mv, max_ma);
                  } else if (sub == 1U) {
                      uint32_t max_mv = ((pdo >> 17U) & 0x1FFU) * 100U;
                      axxpd_request_voltage(max_mv, 0);
                  }
              }
          }
          g_boot_pdo = 0;
      }

      /* PD stack tick (every iteration, ~1-2 ms) */
      axxpd_run();
      axxpd_ensure_contract_flag();  /* repair after hard resets */

      /* Charger disconnect detection — if output is enabled but PD
       * contract is lost (charger unplugged), disable output. */
      if (g_output_enabled && !g_hw_fault && axxpd_get_active_pdo_index() == 0) {
          Output_Disable();
          g_output_enabled = 0;
      }

      /* Tool state machines (selftest, voltage sweep) — need fast ticking */
      UI_ToolTick(&g_ina_reading);

      /* --- Deferred EPR entry (non-blocking) ---
       * If not in EPR and source supports it, attempt EPR entry from the
       * main loop.  Waits ≥1.5 s from boot (chargers like Anker A2697 need
       * settling time after initial SPR contract before accepting EPR).
       * Up to 3 attempts, 2 s apart.  UI stays responsive throughout. */
      /* EPR entry: boot_selector fires at 1s. If it fails (Soft Reset
       * sets EPR_AUTO_ENTER_DISABLED), retry once at 3s. Give up at 5s. */
      if (g_epr_boot_phase < 2) {
          if (axxpd_is_epr_active()) {
              g_epr_boot_phase = 2;
          } else if (g_epr_boot_phase == 0 && (now - boot_tick) >= 3000U
                     && axxpd_is_src_epr_capable()) {
              /* Single retry at 3s if boot_selector's 1s attempt failed */
              axxpd_enable_epr();
              g_epr_boot_phase = 1;
          } else if ((now - boot_tick) >= 5000U) {
              g_epr_boot_phase = 2;
              /* Don't give up on EPR while a "Restore last V/I" is still waiting
               * for an EPR voltage to appear — it drives EPR entry itself. */
              if (!g_boot_restore_vi) {
                  axxpd_disable_epr_intent();
                  axxpd_ensure_contract_flag();
              }
          }
      }

      /* "Restore last V/I" one-shot: once the PD negotiation (incl. EPR) has
       * settled (phase >= 2) and a contract is up, request the saved V/I.
       * Waiting for phase >= 2 means a saved EPR voltage (e.g. 48V) is applied
       * only after EPR has shown up. Does not enable the output. */
      if (g_boot_restore_vi && g_epr_boot_phase >= 2
          && axxpd_get_active_pdo_index() > 0) {
          /* Restore the saved V/I, retrying until it takes.  Two things are
           * required for an EPR voltage (e.g. 48V) and BOTH must happen:
           *   1. Assert EPR intent — axxpd_enable_epr() sets cli_set_epr_intent
           *      + EPR_MODE_ENTRY.  Without it, cli_poll() re-disables EPR
           *      auto-enter (EPR_AUTO_ENTER_DISABLED) and entry is blocked.
           *      (axxpd_request_voltage()'s trigger_any path does NOT set intent
           *      — only its AVS branch does — which is why a bare request stays
           *      at 5V at boot.)
           *   2. Request the PDO — trigger_any performs the actual EPR entry +
           *      PDO selection once intent is asserted.
           * Verified on hardware: this is exactly the state the interactive
           * :SOUR:VOLT 48V relies on.  Some chargers Soft-Reset the first EPR
           * attempt and need a few seconds to settle after the SPR contract, so
           * retry every 2.5s for up to 15s.  If the saved voltage still isn't
           * reached the charger can't supply it → fall back to lowest (vSafe5V).
           * Does not enable the output. */
          static uint32_t s_restore_first = 0;   /* first attempt tick (0=none) */
          static uint32_t s_restore_retry = 0;   /* last attempt tick           */
          uint32_t sv = Settings_GetLastVoltage();
          uint32_t nv = (uint32_t)(axxpd_get_negotiated_v() * 1000.0f);
          if (boot_restore_supported() && (nv + 1500U >= sv)) {
              g_boot_restore_vi = 0;             /* reached saved voltage — done */
          } else if (s_restore_first != 0U
                     && (uint32_t)(now - s_restore_first) >= 15000U) {
              g_boot_restore_vi = 0;             /* charger can't supply → lowest */
              boot_restore_lowest();
          } else if (s_restore_first == 0U
                     || (uint32_t)(now - s_restore_retry) >= 2500U) {
              if (s_restore_first == 0U) s_restore_first = now;
              s_restore_retry = now;
              if (sv > 20000U) axxpd_enable_epr();   /* assert EPR intent first */
              axxpd_request_voltage(sv, Settings_GetLastCurrent());
          }
      }

      /* "Power on at boot" one-shot: arm the output once the PD negotiation
       * (incl. EPR) has settled (phase >= 2) and a valid contract is up with
       * no active fault.  Waits for any pending V/I restore first so it arms at
       * the restored voltage. No voltage cap — gated only by contract + no fault. */
      if (g_boot_power_on && !g_boot_restore_vi && !g_output_enabled && !g_hw_fault
          && g_epr_boot_phase >= 2 && axxpd_get_active_pdo_index() > 0
          && (uint32_t)(now - g_boot_t0) >= (AUTOSTART_COUNTDOWN_S * 1000U)) {
          g_boot_power_on = 0;            /* fire once */
          Output_Enable();
          g_output_enabled = 1;
          if (Settings_GetTimerSeconds() > 0) Timer_Start();
      }

      /* INA228 power monitoring (10 ms / 100 Hz) */
      if ((now - last_ina_ms) >= INA228_SAMPLE_PERIOD_MS) {
          last_ina_ms = now;
          if (INA228_ReadAll(&g_ina, &g_ina_reading) == HAL_OK) {
              g_ina_valid = 1;
              g_ina_fail  = 0;
              /* Session peak tracking for the Energy screen */
              if (g_ina_reading.current_a > g_peak_current_a)
                  g_peak_current_a = g_ina_reading.current_a;
              if (g_ina_reading.power_w > g_peak_power_w)
                  g_peak_power_w = g_ina_reading.power_w;
          } else {
              /* Read failed — the I2C bus may be wedged. Don't trust the stale
               * value (so the screen/protection never act on a frozen reading),
               * and after a few consecutive failures recover the bus so the
               * reading can't stay frozen forever. */
              g_ina_valid = 0;
              if (g_ina_fail < 0xFFFFU) g_ina_fail++;
              if (g_ina_fail >= INA_FAIL_RECOVER_N) {
                  I2C3_BusRecover();
                  g_ina_fail = 0;
                  g_ina_recover_count++;
                  if (g_stream_enabled) {
                      char ebuf[32];
                      int en = snprintf(ebuf, sizeof(ebuf), "#EVT I2C_RECOVER %lu\r\n",
                                        (unsigned long)g_ina_recover_count);
                      if (en > 0) CDC_Transmit_Blocking((const uint8_t *)ebuf, (uint16_t)en, 50);
                  }
              }
          }

          /* Software protection checks (only on a fresh, valid reading) */
          if (g_output_enabled && !g_hw_fault && g_ina_valid &&
              (int32_t)(g_fault_suppress_until - now) <= 0) {
              /* OPP: check power vs settings limit */
              {
                  uint16_t opp = Settings_GetOpp100mw();
                  if (opp > 0U && g_ina_reading.power_w * 10.0f > (float)opp) {
                      g_hw_fault = 1;
                      g_fault_source = FAULT_OPP;
                      Output_Disable();
                      g_fault_pending_beep = 1;
                      FaultLog_Push(FAULT_OPP,
                                    (uint16_t)(g_ina_reading.voltage_v * 1000.0f),
                                    (uint16_t)(g_ina_reading.current_a * 1000.0f));
                  }
              }
              /* Ah limit: accumulated charge exceeds target */
              {
                  uint32_t ah_lim = Settings_GetAhLimitMah();
                  if (ah_lim > 0U && g_ina_reading.charge_ah * 1000.0f >= (float)ah_lim) {
                      g_hw_fault = 1;
                      g_fault_source = FAULT_AH_LIMIT;
                      Output_Disable();
                      g_fault_pending_beep = 1;
                      FaultLog_Push(FAULT_AH_LIMIT,
                                    (uint16_t)(g_ina_reading.voltage_v * 1000.0f),
                                    (uint16_t)(g_ina_reading.current_a * 1000.0f));
                  }
              }
              /* Wh limit: accumulated energy exceeds target */
              {
                  uint32_t wh_lim = Settings_GetWhLimitMwh();
                  if (wh_lim > 0U && g_ina_reading.energy_wh * 1000.0f >= (float)wh_lim) {
                      g_hw_fault = 1;
                      g_fault_source = FAULT_WH_LIMIT;
                      Output_Disable();
                      g_fault_pending_beep = 1;
                      FaultLog_Push(FAULT_WH_LIMIT,
                                    (uint16_t)(g_ina_reading.voltage_v * 1000.0f),
                                    (uint16_t)(g_ina_reading.current_a * 1000.0f));
                  }
              }
              /* Charge-complete: current below threshold for N seconds */
              {
                  uint16_t cc_ma = Settings_GetChargeCompleteMa();
                  uint16_t cc_sec = Settings_GetChargeCompleteSec();
                  if (cc_ma > 0U && g_ina_reading.current_a * 1000.0f < (float)cc_ma) {
                      if (cc_low_since == 0U) cc_low_since = now;
                      if ((now - cc_low_since) >= (uint32_t)cc_sec * 1000U) {
                          g_hw_fault = 1;
                          g_fault_source = FAULT_CHARGE_COMPLETE;
                          Output_Disable();
                          g_fault_pending_beep = 1;
                          FaultLog_Push(FAULT_CHARGE_COMPLETE,
                                        (uint16_t)(g_ina_reading.voltage_v * 1000.0f),
                                        (uint16_t)(g_ina_reading.current_a * 1000.0f));
                          cc_low_since = 0;
                      }
                  } else {
                      cc_low_since = 0;
                  }
              }
          }
      }

      /* Data streaming — prefixed with #S for dashboard parsing */
      if (g_stream_enabled && (now - last_stream_ms) >= g_stream_interval_ms) {
          last_stream_ms = now;
          char sbuf[96];
          /* Temperature conversion for display — protection logic stays in Celsius */
          float stream_tdie = g_ina_reading.die_temp_c;
          float stream_tntc = g_ntc_temp;
          if (Settings_GetTempFahrenheit()) {
              stream_tdie = stream_tdie * 9.0f / 5.0f + 32.0f;
              stream_tntc = stream_tntc * 9.0f / 5.0f + 32.0f;
          }
          snprintf(sbuf, sizeof(sbuf),
                   "#S %.3f,%.3f,%.3f,%.4f,%.4f,%.1f,%.1f,%u\r\n",
                   (double)g_ina_reading.voltage_v,
                   (double)g_ina_reading.current_a,
                   (double)g_ina_reading.power_w,
                   (double)g_ina_reading.energy_wh,
                   (double)g_ina_reading.charge_ah,
                   (double)stream_tdie,
                   (double)stream_tntc,
                   (unsigned)g_output_enabled);
          CDC_Transmit_Blocking((const uint8_t *)sbuf, (uint16_t)strlen(sbuf), 50);
      }

      /* LTC4368 fault detection via EXTI only (no polling).
       * Polling caused false triggers from SPI DMA noise coupling. */

      /* Feed graph samples — checked EVERY loop iteration (not gated by the
       * 33 ms UI refresh) so the window interval is precise: 50/100/200 ms gives
       * exactly a 5/10/20 s window, and the scroll speed tracks the setting.
       * (Nesting this in the 33 ms gate quantized the interval to ~33 ms steps,
       * so the real window was ~6.6/13.2/23 s and didn't match the label.) */
      {
          static uint8_t was_on_graph = 0;
          uint8_t on_graph = (UI_GetScreen() == UI_SCREEN_GRAPH) ? 1 : 0;
          if (on_graph && !was_on_graph) {
              Graph_Init();
              last_graph_sample_ms = now;
          }
          if (on_graph) {
              /* window = 100 pts * interval; 5/10/30/60 s -> 50/100/300/600 ms */
              uint32_t interval_ms;
              switch (Settings_GetGraphWindow()) {
                  case 1:  interval_ms = 100; break;  /* 10s */
                  case 2:  interval_ms = 300; break;  /* 30s */
                  case 3:  interval_ms = 600; break;  /* 60s */
                  default: interval_ms = 50;  break;  /* 5s */
              }
              if ((now - last_graph_sample_ms) >= interval_ms) {
                  last_graph_sample_ms = now;
                  Graph_AddSample(g_ina_reading.voltage_v, g_ina_reading.current_a);
              }
          }
          was_on_graph = on_graph;
      }

      /* UI refresh (33 ms / ~30 Hz) */
      if ((now - last_ui_ms) >= UI_REFRESH_PERIOD_MS) {
          last_ui_ms = now;
          UI_Update(&g_ina_reading, g_ntc_temp, g_output_enabled);
      }

      /* NTC temperature (500 ms / 2 Hz) */
      if ((now - last_ntc_ms) >= NTC_POLL_PERIOD_MS) {
          last_ntc_ms = now;
          g_ntc_temp = NTC_ReadTemperature();

          /* NTC sensor failure — sentinel value means sensor is broken */
          if (g_ntc_temp < -200.0f && g_output_enabled && !g_hw_fault) {
              g_hw_fault = 1;
              g_fault_source = FAULT_THERMAL;
              Output_Disable();
              g_fault_pending_beep = 1;
              FaultLog_Push(FAULT_THERMAL,
                            (uint16_t)(g_ina_reading.voltage_v * 1000.0f),
                            (uint16_t)(g_ina_reading.current_a * 1000.0f));
          }

          /* Thermal warning at 60C — audible chirp, clears with 5C hysteresis */
          {
              static uint8_t thermal_warn_active = 0;
              if (g_ntc_temp >= THERMAL_WARN_C && g_output_enabled && !thermal_warn_active) {
                  thermal_warn_active = 1;
                  Buzzer_Warn();
              }
              if (g_ntc_temp < (THERMAL_WARN_C - 5) && thermal_warn_active) {
                  thermal_warn_active = 0;
              }
          }

          /* Thermal protection — shutdown at 85C */
          if (g_ntc_temp >= THERMAL_SHUTDOWN_C && g_output_enabled && !g_hw_fault) {
              g_hw_fault = 1;
              g_fault_source = FAULT_THERMAL;
              Output_Disable();
              g_fault_pending_beep = 1;
              FaultLog_Push(FAULT_THERMAL,
                            (uint16_t)(g_ina_reading.voltage_v * 1000.0f),
                            (uint16_t)(g_ina_reading.current_a * 1000.0f));
          }
      }

      /* OCP retry: after inrush trip, wait 200ms then re-enable output.
       * If it trips again within 500ms, it's a real fault → latch off.
       * We re-enable manually here instead of calling Output_Enable()
       * because Output_Enable() resets ocp_retry_count and the fault
       * suppression window — that would defeat the retry counter. */
      if (ocp_retry_pending && !g_hw_fault && (int32_t)(now - ocp_retry_tick) >= 200) {
          ocp_retry_pending = 0;
          ocp_retry_count++;
          INA228_ClearAlertLatch(&g_ina);
          g_fault_suppress_until = now + 500U;
          cc_low_since = 0;
          HAL_GPIO_WritePin(BLEED_CTRL_GPIO_Port, BLEED_CTRL_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LTC4368_SHDN_GPIO_Port, LTC4368_SHDN_Pin, GPIO_PIN_SET);
          g_output_enabled = 1;
          if (Settings_GetTimerSeconds() > 0) Timer_Start();
      }

      /* Post-suppression fault pin poll — EXTI is edge-triggered, so if
       * LTC4368_FLT, INA228_ALERT or COMP1 OVP was already asserted when
       * the suppression window expired, no new interrupt fires.  Poll in
       * a 200ms window after the suppression closes to catch faults from
       * e.g. turning on into a dead short or a sustained OVP.
       *
       * The INA228 ALERT pin is LATCHED (ALATCH): a transient that tripped
       * during the suppressed PDO-transition inrush holds the pin low long
       * after the overcurrent is gone (no-load selftest popped a false
       * "OCP (INA228)" this way).  So on the first pass after the window
       * closes, clear the latch and re-check on later passes — a real
       * overcurrent re-asserts ALERT within one conversion (~1 ms).
       * Persistent FLT/ALERT goes through HAL_GPIO_EXTI_Callback so it
       * gets the same OCP retry policy as a live edge (soft-start retry
       * recovers an LTC4368 that latched off on transition inrush) instead
       * of instantly latching a fault screen. */
      {
          static uint8_t post_suppress_rechecked = 0;
          if ((int32_t)(g_fault_suppress_until - now) > 0) {
              post_suppress_rechecked = 0;  /* new suppression window opened */
          } else if (g_output_enabled && !g_hw_fault &&
                     (int32_t)(now - g_fault_suppress_until) < 200U) {
              if (!post_suppress_rechecked) {
                  post_suppress_rechecked = 1;
                  INA228_ClearAlertLatch(&g_ina);  /* drop stale latched ALERT */
              } else if (HAL_GPIO_ReadPin(LTC4368_FLT_GPIO_Port, LTC4368_FLT_Pin) == GPIO_PIN_RESET) {
                  HAL_GPIO_EXTI_Callback(LTC4368_FLT_Pin);
              } else if (HAL_GPIO_ReadPin(INA228_ALERT_GPIO_Port, INA228_ALERT_Pin) == GPIO_PIN_RESET) {
                  HAL_GPIO_EXTI_Callback(INA228_ALERT_Pin);
              }
          }
      }
      if (g_output_enabled && !g_hw_fault &&
          (int32_t)(g_fault_suppress_until - now) <= 0 &&
          (int32_t)(now - g_fault_suppress_until) < 200U) {
          if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUT_LEVEL_HIGH) {
              g_hw_fault = 1;
              g_fault_source = FAULT_COMP_OVP;
              Output_Disable_ISR();
              g_fault_pending_beep = 1;
              FaultLog_Push(FAULT_COMP_OVP,
                            (uint16_t)(g_ina_reading.voltage_v * 1000.0f),
                            (uint16_t)(g_ina_reading.current_a * 1000.0f));
          }
      }

      /* Button event processing */
      Buttons_Update();
      {
          ButtonEvent_t ev = Buttons_GetEvent();
          if (ev != BTN_NONE && ev != BTN_PWR_LONG
              && ev != BTN_INC_REPEAT && ev != BTN_DEC_REPEAT) {
              Buzzer_Click();
          }
          if ((ev == BTN_PWR_LONG || ev == BTN_PWR_SHORT) && g_hw_fault) {
              /* Fault active — only SELECT can clear it. Use the audible
               * turn-off tone for the rejected ON/OFF press. */
              Buzzer_Disable();
          } else if (ev == BTN_PWR_LONG && !UI_IsLocked()) {
              g_output_enabled = 0;
              Output_Disable();
              Buzzer_Disable();
          } else if (ev == BTN_PWR_SHORT && !UI_IsLocked()) {
              if (UI_WantsPwrShort()) {
                  UI_HandleButton(ev);
              } else {
                  if (!g_output_enabled && g_ntc_temp >= THERMAL_COOLDOWN_C) {
                      /* Still too hot — don't allow re-enable until the board
                       * has cooled below the cooldown threshold. Checked on
                       * temperature alone (not fault bookkeeping): clearing a
                       * thermal fault also resets g_fault_source, and an
                       * unplugged NTC reads -273 so it never blocks here. */
                      Buzzer_Fault();
                  } else if (!g_output_enabled && UI_GetScreen() == UI_SCREEN_SETTINGS) {
                      /* Can't arm the output from the settings screen — give the
                       * normal, audible turn-off tone (the short fault chirp here
                       * was too quiet to notice). */
                      Buzzer_Disable();
                  } else if (!g_output_enabled &&
                             (int32_t)(HAL_GetTick() - g_output_enable_tick) < (int32_t)OUTPUT_TOGGLE_COOLDOWN_MS) {
                      /* Too soon after last enable — protect MOSFETs from
                       * thermal stress caused by repeated gate ramp cycles.
                       * Use the audible turn-off tone (the short fault chirp
                       * was too quiet to notice). */
                      Buzzer_Disable();
                  } else {
                      __disable_irq();
                      g_output_enabled = !g_output_enabled;
                      uint8_t en = g_output_enabled;
                      __enable_irq();
                      if (en) { Output_Enable(); Buzzer_Enable(); }
                      else    { Output_Disable(); Buzzer_Disable();
                                /* Clear any fault triggered by the discharge
                                 * transient (COMP1 OVP, LTC4368 FLT).  The
                                 * user intentionally turned off the output —
                                 * the transient is not a real fault. */
                                g_hw_fault = 0; g_fault_source = FAULT_NONE;
                                INA228_ClearAlertLatch(&g_ina); }
                  }
              }
          } else if (ev != BTN_NONE) {
              UI_HandleButton(ev);
          }
      }

      /* Deferred settings save from ISR-context Output_Disable_ISR() */
      if (g_save_on_disable) {
          g_save_on_disable = 0;
          uint32_t lv = (uint32_t)(axxpd_get_negotiated_v() * 1000.0f);
          uint32_t la = (uint32_t)(axxpd_get_negotiated_a() * 1000.0f);
          if (lv >= 3300) Settings_SaveLastSettings(lv, la);
      }

      /* Deferred flash writes (EPR-safe: skips when PD is in EPR KeepAlive) */
      Settings_ProcessDeferred();

      /* Deferred fault beep */
      if (g_fault_pending_beep) {
          g_fault_pending_beep = 0;
          Buzzer_Fault();  /* 2kHz fault tone — bypasses the buzzer setting (safety critical) */
      }

      /* Timer auto-shutoff */
      if (timer_running && (int32_t)(now - timer_end_tick) >= 0) {
          timer_running = 0;
          g_hw_fault = 1;
          g_fault_source = FAULT_TIMER;
          Output_Disable();
          FaultLog_Push(FAULT_TIMER, 0, 0);
          g_fault_pending_beep = 1;
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 64;  /* 16/4*64/2 = 128MHz */
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;  /* PCLK1=16MHz for UCPD */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  /* PCLK2=128MHz for SPI */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO2;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH1;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_MEDIUM;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x40B285C2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;  /* 128MHz/4 = 32MHz */
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 127;  /* 128MHz/128 = 1MHz buzzer timer clock */
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief UCPD1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UCPD1_Init(void)
{

  /* USER CODE BEGIN UCPD1_Init 0 */

  /* USER CODE END UCPD1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**UCPD1 GPIO Configuration
  PB4   ------> UCPD1_CC2
  PB6   ------> UCPD1_CC1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* UCPD1 DMA Init */

  /* UCPD1_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_UCPD1_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

  /* UCPD1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_UCPD1_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* UCPD1 interrupt Init */
  NVIC_SetPriority(UCPD1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(UCPD1_IRQn);

  /* USER CODE BEGIN UCPD1_Init 1 */

  /* USER CODE END UCPD1_Init 1 */
  /* USER CODE BEGIN UCPD1_Init 2 */

  /* USER CODE END UCPD1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration — SPI1 TX for LCD */
  NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TFT_DC_Pin, GPIO_PIN_RESET);
  /* BLEED_CTRL HIGH at boot — actively discharge output caps while output
   * is off.  Output_Enable() will clear it when the output is turned on. */
  HAL_GPIO_WritePin(BLEED_CTRL_GPIO_Port, BLEED_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level — LTC4368 SHDN low = output disabled at boot */
  HAL_GPIO_WritePin(LTC4368_SHDN_GPIO_Port, LTC4368_SHDN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LTC4368_SHDN_Pin (PA1) — push-pull output for output switch */
  GPIO_InitStruct.Pin = LTC4368_SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LTC4368_SHDN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : V_BIT_1_Pin V_BIT_2_Pin V_BIT_3_Pin */
  GPIO_InitStruct.Pin = V_BIT_1_Pin|V_BIT_2_Pin|V_BIT_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TPD4S480_FLT_Pin — active-low, detect fault onset */
  GPIO_InitStruct.Pin = TPD4S480_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TPD4S480_FLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LTC4368_FLT_Pin — active-low open-drain, detect fault onset */
  GPIO_InitStruct.Pin = LTC4368_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LTC4368_FLT_GPIO_Port, &GPIO_InitStruct);

  /* Enable EXTI NVIC for all fault pins */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);       /* TPD4S480_FLT (PF1) */
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);       /* LTC4368_FLT (PA3) */
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);     /* LM5166_FGOOD (PB9) */
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);   /* INA228_ALERT (PB11) */
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /*Configure GPIO pin : TFT_CS_Pin */
  GPIO_InitStruct.Pin = TFT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TFT_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_DC_Pin BLEED_CTRL_Pin */
  GPIO_InitStruct.Pin = TFT_DC_Pin|BLEED_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TFT_RESET_Pin */
  GPIO_InitStruct.Pin = TFT_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TFT_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INA228_ALERT_Pin LM5166_FGOOD_Pin — fault EXTI */
  /* Both are open-drain outputs on their respective ICs — pull-up required */
  GPIO_InitStruct.Pin = INA228_ALERT_Pin|LM5166_FGOOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_1_Pin SW_2_Pin SW_3_Pin SW_4_Pin — polled buttons */
  GPIO_InitStruct.Pin = SW_1_Pin|SW_2_Pin|SW_3_Pin|SW_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ---------------------------------------------------------------------------
 * Output Enable / Disable — controls LTC4368 via PA1 (LTC4368_SHDN).
 * High = enabled (SHDN de-asserted), Low = disabled (SHDN asserted).
 * Switched from TIM15 CH1N PWM to plain GPIO for reliable bringup.
 * ---------------------------------------------------------------------------*/
void Output_Enable(void) {
    g_fault_suppress_until = HAL_GetTick() + 1000U;  /* 1 s inrush window */
    g_hw_fault = 0;
    g_fault_source = FAULT_NONE;
    ocp_retry_count = 0;
    ocp_retry_pending = 0;
    cc_low_since = 0;
    HAL_GPIO_WritePin(BLEED_CTRL_GPIO_Port, BLEED_CTRL_Pin, GPIO_PIN_RESET);
    /* Drive SHDN high via PA1 GPIO — reconfigure from TIM15 AF to push-pull */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = LTC4368_SHDN_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LTC4368_SHDN_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(LTC4368_SHDN_GPIO_Port, LTC4368_SHDN_Pin, GPIO_PIN_SET);
    g_output_enabled = 1;
    g_output_enable_tick = HAL_GetTick();
    if (Settings_GetTimerSeconds() > 0) Timer_Start();
}

/* Guarded enable for remote (CLI/SCPI) callers — enforces the same
 * re-enable protections as the POWER button path, which were previously
 * bypassed when controlling the output over serial:
 *   0 = enabled OK
 *   1 = blocked by the MOSFET toggle cooldown (1.5 s between enables,
 *       prevents gate-ramp thermal stress from rapid on/off cycling)
 *   2 = blocked by thermal cooldown (board still above 75 C) */
uint8_t Output_Enable_Guarded(void) {
    if (!g_output_enabled && g_ntc_temp >= THERMAL_COOLDOWN_C)
        return 2;
    if (!g_output_enabled &&
        (int32_t)(HAL_GetTick() - g_output_enable_tick) < (int32_t)OUTPUT_TOGGLE_COOLDOWN_MS)
        return 1;
    Output_Enable();
    return 0;
}

/* ISR-safe output disable — GPIO + bleed only, no flash writes.
 * Safe to call from COMP1/EXTI ISR context. Sets g_save_on_disable
 * so the main loop can do the deferred settings save. */
void Output_Disable_ISR(void) {
    g_fault_suppress_until = HAL_GetTick() + 500U;
    /* Drive SHDN low via PA1 GPIO */
    HAL_GPIO_WritePin(LTC4368_SHDN_GPIO_Port, LTC4368_SHDN_Pin, GPIO_PIN_RESET);
    /* Enable bleed resistor to discharge output caps */
    HAL_GPIO_WritePin(BLEED_CTRL_GPIO_Port, BLEED_CTRL_Pin, GPIO_PIN_SET);
    g_output_enabled = 0;
    Timer_Stop();
    g_save_on_disable = 1;
}

void Output_Disable(void) {
    /* Suppress fault ISRs during discharge transient — the LM5166 PGOOD
     * and COMP1 OVP can glitch as the output caps discharge. */
    g_fault_suppress_until = HAL_GetTick() + 500U;
    /* Drive SHDN low via PA1 GPIO */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = LTC4368_SHDN_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LTC4368_SHDN_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(LTC4368_SHDN_GPIO_Port, LTC4368_SHDN_Pin, GPIO_PIN_RESET);
    /* Enable bleed resistor to discharge output caps */
    HAL_GPIO_WritePin(BLEED_CTRL_GPIO_Port, BLEED_CTRL_Pin, GPIO_PIN_SET);
    g_output_enabled = 0;
    Timer_Stop();
    /* Save last voltage/current for remember-boot feature */
    {
        uint32_t lv = (uint32_t)(axxpd_get_negotiated_v() * 1000.0f);
        uint32_t la = (uint32_t)(axxpd_get_negotiated_a() * 1000.0f);
        if (lv >= 3300) Settings_SaveLastSettings(lv, la);
    }
}

/* ---------------------------------------------------------------------------
 * OVP — software-adjustable threshold via DAC1_CH1 → COMP1_INM
 * V_SENSE (PB1) feeds COMP1_INP through a resistor divider.
 * DAC sets the reference voltage that COMP1 compares against.
 * ---------------------------------------------------------------------------*/
void OVP_SetThreshold(uint32_t vbus_max_mv) {
    /* DAC output voltage = vbus_max_mv * divider_ratio
     * DAC code = (dac_voltage / VREF) * 4095
     * VREF = 3.3V on G4 */
    float dac_v = (float)vbus_max_mv / 1000.0f * OVP_DIVIDER_RATIO;
    uint32_t code = (uint32_t)(dac_v / 3.3f * 4095.0f + 0.5f);
    if (code > 4095) code = 4095;
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, code);
}

/* COMP1 trigger callback — called when V_SENSE > DAC threshold (OVP trip) */
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
    if (hcomp->Instance == COMP1) {
        if (g_output_enabled && !g_hw_fault &&
            (int32_t)(g_fault_suppress_until - HAL_GetTick()) <= 0) {
            g_hw_fault = 1;
            g_fault_source = FAULT_COMP_OVP;
            Output_Disable_ISR();
            g_fault_pending_beep = 1;
            FaultLog_Push(FAULT_COMP_OVP,
                          (uint16_t)(g_ina_reading.voltage_v * 1000.0f),
                          (uint16_t)(g_ina_reading.current_a * 1000.0f));
        }
    }
}

/* EXTI callback — LTC4368 fault, INA228 alert, LM5166 power-good, TPD4S480 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    /* Suppress ALL faults during inrush/PDO transition window */
    if ((int32_t)(g_fault_suppress_until - HAL_GetTick()) > 0) return;

    if (GPIO_Pin == LTC4368_FLT_Pin) {
        /* Re-read pin to reject noise glitches */
        if (HAL_GPIO_ReadPin(LTC4368_FLT_GPIO_Port, LTC4368_FLT_Pin) != GPIO_PIN_RESET)
            return;
        if (g_output_enabled && !g_hw_fault) {
            uint8_t max_retries = Settings_GetOcpRetry();  /* 0=latch, 1=1x, 2=3x */
            uint8_t allowed = (max_retries == 0) ? 0
                            : (max_retries == 1) ? 1 : 3;
            uint32_t now = HAL_GetTick();

            /* If a recent OCP retry succeeded (>500ms since last trip),
             * reset the counter — the previous trip was just inrush. */
            if (ocp_retry_count > 0 && (now - ocp_retry_tick) > 500U) {
                ocp_retry_count = 0;
            }

            if (ocp_retry_count < allowed) {
                /* Inrush likely — cut SHDN but keep bleed OFF so output caps
                 * stay charged.  This way the retry re-enable sees a smaller
                 * voltage delta to the load, reducing inrush on the next try. */
                HAL_GPIO_WritePin(LTC4368_SHDN_GPIO_Port, LTC4368_SHDN_Pin, GPIO_PIN_RESET);
                g_output_enabled = 0;
                ocp_retry_tick = now;
                ocp_retry_pending = 1;
                /* Don't set g_hw_fault — no error screen yet */
            } else {
                /* Real fault or retries exhausted — latch off permanently */
                g_hw_fault = 1;
                g_fault_source = FAULT_LTC4368;
                Output_Disable_ISR();
                g_fault_pending_beep = 1;
                ocp_retry_count = 0;
                ocp_retry_pending = 0;
                FaultLog_Push(FAULT_LTC4368, 0, 0);
            }
        }
        return;
    }

    if (GPIO_Pin == INA228_ALERT_Pin) {
        if (g_output_enabled && !g_hw_fault) {
            uint8_t max_retries = Settings_GetOcpRetry();
            uint8_t allowed = (max_retries == 0) ? 0
                            : (max_retries == 1) ? 1 : 3;
            uint32_t now = HAL_GetTick();

            if (ocp_retry_count > 0 && (now - ocp_retry_tick) > 500U) {
                ocp_retry_count = 0;
            }

            if (ocp_retry_count < allowed) {
                HAL_GPIO_WritePin(LTC4368_SHDN_GPIO_Port, LTC4368_SHDN_Pin, GPIO_PIN_RESET);
                g_output_enabled = 0;
                ocp_retry_tick = now;
                ocp_retry_pending = 1;
            } else {
                g_hw_fault = 1;
                g_fault_source = FAULT_INA228_OCP;
                Output_Disable_ISR();
                g_fault_pending_beep = 1;
                ocp_retry_count = 0;
                ocp_retry_pending = 0;
                FaultLog_Push(FAULT_INA228_OCP, 0, 0);
            }
        }
    }
    else if (GPIO_Pin == LM5166_FGOOD_Pin) {
        /* PGOOD is informational only — if the 3.3V rail truly fails, the
         * MCU's built-in BOR resets it automatically.  Brief PGOOD glitches
         * during PDO transitions and PFM mode switches are harmless and
         * only cause false fault screens.  Ignore entirely. */
        (void)0;
    }
    else if (GPIO_Pin == TPD4S480_FLT_Pin) {
        if (g_output_enabled && !g_hw_fault) {
            g_hw_fault = 1;
            g_fault_source = FAULT_TPD4S480;
            Output_Disable_ISR();
            g_fault_pending_beep = 1;
            FaultLog_Push(FAULT_TPD4S480, 0, 0);
        }
    }
}

/* ---------------------------------------------------------------------------
 * Fault log — ring buffer
 * ---------------------------------------------------------------------------*/
void FaultLog_Push(uint8_t source, uint16_t mv, uint16_t ma) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t idx = g_fault_log_head;
    g_fault_log[idx].tick = HAL_GetTick();
    g_fault_log[idx].source = source;
    g_fault_log[idx].voltage_mv = mv;
    g_fault_log[idx].current_ma = ma;
    g_fault_log_head = (idx + 1) % FAULT_LOG_SIZE;
    if (g_fault_log_count < FAULT_LOG_SIZE) g_fault_log_count++;
    __set_PRIMASK(primask);
}

FaultLog_t* FaultLog_Get(uint8_t index) {
    if (index >= g_fault_log_count) return 0;
    uint8_t actual = (g_fault_log_head - g_fault_log_count + index + FAULT_LOG_SIZE) % FAULT_LOG_SIZE;
    return (FaultLog_t*)&g_fault_log[actual];
}

uint8_t FaultLog_Count(void) {
    return g_fault_log_count;
}

/* ---------------------------------------------------------------------------
 * Timer auto-shutoff
 * ---------------------------------------------------------------------------*/
void Timer_Start(void) {
    uint16_t seconds = Settings_GetTimerSeconds();
    if (seconds == 0) return;
    timer_end_tick = HAL_GetTick() + (uint32_t)seconds * 1000U;
    timer_running = 1;
}

void Timer_Stop(void) {
    timer_running = 0;
}

uint16_t Timer_GetRemaining(void) {
    if (!timer_running) return 0;
    int32_t rem = (int32_t)(timer_end_tick - HAL_GetTick());
    if (rem <= 0) return 0;
    return (uint16_t)(rem / 1000U);
}

/* ---------------------------------------------------------------------------
 * Hardware version detection — reads PC13/PC14/PC15 straps
 * ---------------------------------------------------------------------------*/
uint8_t get_hw_version(void) {
    uint8_t v = 0;
    if (HAL_GPIO_ReadPin(VERSION_BIT_1_GPIO_Port, VERSION_BIT_1_Pin) == GPIO_PIN_SET) v |= 1;
    if (HAL_GPIO_ReadPin(VERSION_BIT_2_GPIO_Port, VERSION_BIT_2_Pin) == GPIO_PIN_SET) v |= 2;
    if (HAL_GPIO_ReadPin(VERSION_BIT_3_GPIO_Port, VERSION_BIT_3_Pin) == GPIO_PIN_SET) v |= 4;
    return v;
}

void App_SetTargetVoltage(uint32_t mv, uint32_t ma)
{
    /* Persist the user's selection so "Restore last V/I" can bring it back on
     * the next boot.  This wrapper is only called from user-initiated UI
     * selections (dashboard edit-confirm, preset recall); boot auto-select,
     * the restore poll, the voltage sweep and self-test all call
     * axxpd_request_voltage() directly and therefore never clobber the saved
     * value.  Saving here (before the request) means a 48V/EPR selection made
     * while still on the old non-EPR contract commits to flash immediately;
     * once EPR is active the write defers (10s timeout / EPR drop). */
    if (mv >= 3300U) Settings_SaveLastSettings(mv, ma);
    axxpd_request_voltage(mv, ma);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  /* Start IWDG if not already running, so device resets instead of hanging */
  IWDG->KR  = 0x5555U;
  IWDG->PR  = 4U;
  IWDG->RLR = 100U;  /* Short timeout for quick reset */
  IWDG->KR  = 0xCCCCU;
  while (1) { }  /* IWDG will reset the device */
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
