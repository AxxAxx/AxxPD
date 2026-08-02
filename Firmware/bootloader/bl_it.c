/**
 * @file    bl_it.c
 * @brief   Bootloader interrupt handlers. Faults reset instead of hanging —
 *          the boot decision in main() then lands back in bootloader mode
 *          when the app is invalid, so a fault can never lock the device.
 */

#include "stm32g4xx_hal.h"
#include "bl_layout.h"

extern PCD_HandleTypeDef hpcd_USB_FS;

/* Flash ECC double-error NMI: a torn write/erase (power loss mid-update)
 * leaves a doubleword whose READ raises NMI. A plain reset would re-read it
 * while validating the app and loop forever — the one true brick vector.
 * Instead: clear the latch, arm the request mailbox so the next boot goes
 * STRAIGHT to bootloader mode without touching app flash, and reset. */
void NMI_Handler(void)
{
    if (FLASH->ECCR & FLASH_ECCR_ECCD) {
        FLASH->ECCR |= FLASH_ECCR_ECCD;
        volatile uint32_t *req = (volatile uint32_t *)BL_REQ_ADDR;
        req[0] = BL_REQ_MAGIC;
        req[1] = ~BL_REQ_MAGIC;
        __DSB();
    }
    NVIC_SystemReset();
}
void HardFault_Handler(void)  { NVIC_SystemReset(); }

void MemManage_Handler(void)  { NVIC_SystemReset(); }
void BusFault_Handler(void)   { NVIC_SystemReset(); }
void UsageFault_Handler(void) { NVIC_SystemReset(); }

void SVC_Handler(void)      {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void)   {}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void USB_LP_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}
