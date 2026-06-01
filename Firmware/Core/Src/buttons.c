/**
 * @file    buttons.c
 * @brief   Polling-based button driver with per-button state machine.
 *
 * Based on the AxxSolder button pattern: pure GPIO polling from a 5ms
 * SysTick tick, no EXTI for buttons. Eliminates edge-triggered
 * double-fire issues entirely.
 *
 * Buttons are active-high with external 10k pull-down + 100nF RC debounce.
 * INC/DEC are physically swapped (INC button = DEC action and vice versa).
 *
 * Event types:
 *   INC/DEC/SEL: press event after 20ms debounce, repeat while held (INC/DEC)
 *   PWR: short press on release (<600ms), long press at 600ms threshold
 */

#include "buttons.h"
#include "main.h"

/* Fault pin globals — still handled via EXTI falling edge.
 * These live in main.c and are shared with the fault EXTI callback at the
 * bottom of this file plus the main-loop fault handler.  All are volatile
 * because they are written from ISR context. */
extern volatile uint8_t g_output_enabled;
extern volatile uint8_t g_hw_fault;
extern volatile uint8_t g_fault_source;       /* sequential enum (use == not &) */
extern volatile uint8_t g_fault_pending_beep;
extern volatile uint32_t g_fault_suppress_until;  /* HAL_GetTick() deadline — ignore faults until then */

/* -------------------------------------------------------------------------
 * Timing constants (in 5ms ticks)
 * ---------------------------------------------------------------------- */
/* All timing values are in units of the 5ms SysTick polling interval. */
#define DEBOUNCE_TICKS        2U    /* 10ms  — PCB has hardware RC debounce, software can be short */
#define LONG_PRESS_TICKS      120U  /* 600ms — threshold for PWR/SEL long-press */
#define REPEAT_INIT_TICKS     60U   /* 300ms — hold delay before first auto-repeat (INC/DEC) */
#define REPEAT_INTERVAL_TICKS 16U   /* 80ms  — interval between subsequent repeats */

/* -------------------------------------------------------------------------
 * Per-button state machine
 *
 * Transitions:
 *   IDLE --[pin high]--> DEBOUNCE --[stable DEBOUNCE_TICKS]--> PRESSED
 *   DEBOUNCE --[pin low]--> IDLE  (bounce/noise rejected)
 *   PRESSED --[release]--> IDLE   (fires short-press for PWR/SEL)
 *   PRESSED --[hold timeout]--> LONG_FIRED  (fires long-press or first repeat)
 *   LONG_FIRED --[release]--> IDLE
 *   LONG_FIRED --[still held, INC/DEC]--> stays, fires repeat every 80ms
 * ---------------------------------------------------------------------- */
typedef enum {
    ST_IDLE = 0,
    ST_DEBOUNCE,        /* rising edge detected, confirming */
    ST_PRESSED,         /* confirmed press, timing long/repeat */
    ST_LONG_FIRED,      /* long press or repeat mode active */
} BtnState_t;

typedef struct {
    GPIO_TypeDef   *port;
    uint16_t        pin;
    BtnState_t      state;
    uint16_t        ticks;          /* ticks in current sub-state */
    uint16_t        hold_ticks;     /* total ticks since confirmed press */
    ButtonEvent_t   ev_press;       /* event after debounce (or on release for PWR/SEL) */
    ButtonEvent_t   ev_long;        /* event at long-press threshold (BTN_NONE = disabled) */
    ButtonEvent_t   ev_repeat;      /* auto-repeat event while held (BTN_NONE = disabled) */
    uint8_t         press_on_confirm; /* 1: fire ev_press immediately at debounce (INC/DEC)
                                       * 0: fire ev_press on release only (PWR/SEL) — allows
                                       *    distinguishing short vs long press */
} BtnCtx_t;

static BtnCtx_t s_btns[4];

/* -------------------------------------------------------------------------
 * Event ring buffer
 *
 * Lock-free SPSC (single-producer, single-consumer) FIFO.
 * Producer: Buttons_Tick() running in SysTick ISR context.
 * Consumer: Buttons_GetEvent() called from the main loop.
 * No critical section needed because head is only written by the producer
 * and tail is only written by the consumer.
 * ---------------------------------------------------------------------- */
#define EVT_BUF_SIZE 8U
static volatile ButtonEvent_t s_evt_buf[EVT_BUF_SIZE];
static volatile uint8_t s_evt_head = 0U;  /* next write index (producer) */
static volatile uint8_t s_evt_tail = 0U;  /* next read  index (consumer) */

static void evt_push(ButtonEvent_t ev)
{
    uint8_t next = (s_evt_head + 1U) % EVT_BUF_SIZE;
    if (next != s_evt_tail) {          /* drop oldest-unread if full */
        s_evt_buf[s_evt_head] = ev;
        s_evt_head = next;
    }
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

void Buttons_Init(void)
{
    /* INC/DEC: immediate press event + auto-repeat while held.
     * No long-press event — holding just repeats the increment/decrement.
     *
     * SEL/PWR: deferred press event (fires on release for short-press).
     * Long-press fires at the 600ms threshold while still held.
     * No auto-repeat — these are modal actions, not value adjusters. */

    /* INCREASE (SW2/PB13) — fires immediately, repeats while held */
    s_btns[0] = (BtnCtx_t){
        .port = BUTTON_INCREASE_GPIO_Port, .pin = BUTTON_INCREASE_Pin,
        .ev_press = BTN_INC_PRESS, .ev_long = BTN_NONE,
        .ev_repeat = BTN_INC_REPEAT, .press_on_confirm = 1U
    };
    /* DECREASE (SW3/PB14) — fires immediately, repeats while held */
    s_btns[1] = (BtnCtx_t){
        .port = BUTTON_DECREASE_GPIO_Port, .pin = BUTTON_DECREASE_Pin,
        .ev_press = BTN_DEC_PRESS, .ev_long = BTN_NONE,
        .ev_repeat = BTN_DEC_REPEAT, .press_on_confirm = 1U
    };
    /* SELECT (SW1/PB12) — short press on release, long press at 600ms */
    s_btns[2] = (BtnCtx_t){
        .port = BUTTON_SELECT_GPIO_Port, .pin = BUTTON_SELECT_Pin,
        .ev_press = BTN_SEL_PRESS, .ev_long = BTN_SEL_LONG,
        .ev_repeat = BTN_NONE, .press_on_confirm = 0U
    };
    /* POWER (SW4/PB15) — short press on release, long press at 600ms */
    s_btns[3] = (BtnCtx_t){
        .port = BUTTON_POWER_GPIO_Port, .pin = BUTTON_POWER_Pin,
        .ev_press = BTN_PWR_SHORT, .ev_long = BTN_PWR_LONG,
        .ev_repeat = BTN_NONE, .press_on_confirm = 0U
    };
}

/**
 * @brief  Run all 4 button state machines. Call every 5ms from SysTick.
 */
void Buttons_Tick(void)
{
    for (uint8_t i = 0U; i < 4U; i++) {
        BtnCtx_t *b = &s_btns[i];
        uint8_t high = (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_SET);

        switch (b->state) {

        case ST_IDLE:
            if (high) {
                b->state = ST_DEBOUNCE;
                b->ticks = 0U;
            }
            break;

        case ST_DEBOUNCE:
            if (!high) {
                /* Bounce or noise — discard */
                b->state = ST_IDLE;
            } else if (++b->ticks >= DEBOUNCE_TICKS) {
                /* Confirmed press */
                b->state = ST_PRESSED;
                b->hold_ticks = 0U;
                if (b->press_on_confirm) {
                    evt_push(b->ev_press);
                }
            }
            break;

        case ST_PRESSED:
            b->hold_ticks++;
            if (!high) {
                /* Released */
                if (!b->press_on_confirm) {
                    /* PWR: short press fires on release */
                    evt_push(b->ev_press);
                }
                b->state = ST_IDLE;
            } else if (b->ev_long != BTN_NONE &&
                       b->hold_ticks >= LONG_PRESS_TICKS) {
                /* Long press threshold reached (PWR) */
                evt_push(b->ev_long);
                b->state = ST_LONG_FIRED;
                b->ticks = 0U;
            } else if (b->ev_repeat != BTN_NONE &&
                       b->hold_ticks >= REPEAT_INIT_TICKS) {
                /* First repeat (INC/DEC held long enough) */
                evt_push(b->ev_repeat);
                b->state = ST_LONG_FIRED;
                b->ticks = 0U;
            }
            break;

        case ST_LONG_FIRED:
            if (!high) {
                /* Released — no additional event */
                b->state = ST_IDLE;
            } else if (b->ev_repeat != BTN_NONE) {
                /* Auto-repeat mode (INC/DEC) */
                if (++b->ticks >= REPEAT_INTERVAL_TICKS) {
                    b->ticks = 0U;
                    evt_push(b->ev_repeat);
                }
            }
            /* PWR long: just wait for release, no repeat */
            break;
        }
    }
}

void Buttons_Update(void)
{
    /* No-op — state machines run from Buttons_Tick() in SysTick.
     * Kept for API compatibility with boot_selector.c and main.c. */
}

ButtonEvent_t Buttons_GetEvent(void)
{
    if (s_evt_head == s_evt_tail) return BTN_NONE;
    ButtonEvent_t ev = s_evt_buf[s_evt_tail];
    s_evt_tail = (s_evt_tail + 1U) % EVT_BUF_SIZE;
    return ev;
}

/* -------------------------------------------------------------------------
 * EXTI callbacks — REMOVED
 *
 * Buttons are polled from SysTick (see above). Fault EXTI handlers are
 * in main.c (HAL_GPIO_EXTI_Callback). The STM32 HAL only dispatches to
 * HAL_GPIO_EXTI_Callback — Rising/Falling variants are never called.
 * ---------------------------------------------------------------------- */
