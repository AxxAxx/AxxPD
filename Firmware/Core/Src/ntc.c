/**
 * @file    ntc.c
 * @brief   NTC thermistor temperature driver via ADC2 (PA0).
 *
 * Voltage divider: 3.3V -> 10K pullup -> ADC pin -> NTC 10K (B=3380K) -> GND
 *
 *   R_ntc = 10000 * raw / (4095 - raw)
 *   1/T   = 1/T0 + (1/B) * ln(R_ntc / R0)   [T in Kelvin]
 *
 * ADC2 is pre-configured by CubeMX (12-bit, single conversion).
 * HAL_ADCEx_Calibration_Start() is run once at init (G4 requires ADC_SINGLE_ENDED).
 */

#include "ntc.h"
#include <math.h>

/* -------------------------------------------------------------------------
 * NTC parameters
 * ---------------------------------------------------------------------- */
#define NTC_R0       10000.0f   /* Nominal resistance at T0 (Ohm) */
#define NTC_T0       298.15f    /* Nominal temperature (K = 25 C)  */
#define NTC_BETA     3380.0f    /* Beta constant (K)                */
#define NTC_RPULLUP  10000.0f   /* Pull-up resistor value (Ohm)     */
#define NTC_VREF_ADC 4095.0f    /* 12-bit full-scale count           */

#define ADC_TIMEOUT_MS  10U

static ADC_HandleTypeDef *s_hadc = NULL;

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

void NTC_Init(ADC_HandleTypeDef *hadc)
{
    s_hadc = hadc;
    HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
}

float NTC_ReadTemperature(void)
{
    if (s_hadc == NULL) {
        return -273.15f;
    }

    HAL_ADC_Start(s_hadc);

    if (HAL_ADC_PollForConversion(s_hadc, ADC_TIMEOUT_MS) != HAL_OK) {
        HAL_ADC_Stop(s_hadc);
        return -273.15f;   /* ADC timeout — caller (main.c) should treat -273.15 as "sensor fault" */
    }

    uint32_t raw = HAL_ADC_GetValue(s_hadc);
    HAL_ADC_Stop(s_hadc);

    /* Guard rail values: open circuit or short circuit */
    if (raw == 0U) {
        return -273.15f;   /* NTC open — ADC pulled high, raw near 0 shouldn't happen;
                              but if ADC reads 0, R_ntc = 0 => div-by-zero guard.
                              Caller (main.c) should treat -273.15 as "sensor fault". */
    }
    if (raw >= 4095U) {
        return -273.15f;   /* NTC short or pullup open — return sentinel so main.c
                              treats it as sensor fault (same as ADC timeout).
                              Also prevents division by zero: (NTC_VREF_ADC - raw) = 0. */
    }

    /* R_ntc from voltage divider */
    float r_ntc = NTC_RPULLUP * (float)raw / (NTC_VREF_ADC - (float)raw);

    /* Physically impossible resistance = broken sensor, not a temperature.
     * A nearly-open NTC (cold joint, cracked part) reads as megaohms, which
     * the beta equation maps to a plausible-looking arctic temperature —
     * silently disarming thermal protection. 1 MOhm corresponds to roughly
     * -60 C and 100 Ohm to far beyond +200 C; nothing on this board can be
     * there for real, so treat both extremes as a sensor fault. */
    if (r_ntc > 1.0e6f || r_ntc < 100.0f) {
        return -273.15f;
    }

    /* Steinhart-Hart simplified (beta equation) */
    float inv_t = (1.0f / NTC_T0) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0);
    float temp_k = 1.0f / inv_t;

    return temp_k - 273.15f;
}
