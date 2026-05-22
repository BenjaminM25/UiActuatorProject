/*
 * bootloader.cpp
 *
 *  Created on: 15. apr. 2026
 *      Author: Benjamin
 */


#include "main.h"
#include "bootloader.h"

#define BOOTLOADER_MAGIC_ADDR  (TAMP->BKP0R)
#define BOOTLOADER_MAGIC_VAL   0xDEADBEEF
#define DOUBLE_RESET_MAGIC_VAL 0xCAFEBABE   // distinct value for double-reset
#define SYSTEM_MEMORY_ADDR     0x1FFF0000UL
#define DOUBLE_RESET_WINDOW_MS 500

#define IS_DEBUGGER_ATTACHED() (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)

void checkAndJumpToBootloader(void) {
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_RTC_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    // Path 1: explicit request via requestBootloader() — always honour
    if (BOOTLOADER_MAGIC_ADDR == BOOTLOADER_MAGIC_VAL) {
        BOOTLOADER_MAGIC_ADDR = 0;
        jumpToBootloader();
    }

    // Path 2: double-reset — skip entirely if debugger is attached
    if (!IS_DEBUGGER_ATTACHED()) {
        if (BOOTLOADER_MAGIC_ADDR == DOUBLE_RESET_MAGIC_VAL) {
            BOOTLOADER_MAGIC_ADDR = 0;
            jumpToBootloader();
        }

        BOOTLOADER_MAGIC_ADDR = DOUBLE_RESET_MAGIC_VAL;
        SystemCoreClockUpdate();
        HAL_InitTick(TICK_INT_PRIORITY);
        HAL_Delay(DOUBLE_RESET_WINDOW_MS);
        BOOTLOADER_MAGIC_ADDR = 0;
    } else {
        // Debugger attached — clear any stale magic value and boot normally
        BOOTLOADER_MAGIC_ADDR = 0;
    }
}

// Shared jump logic used by both paths
void jumpToBootloader(void) {
	// --- Tear everything down cleanly ---
	__disable_irq();
	HAL_RCC_DeInit();

	// Reset SysTick
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL  = 0;

	// Disable and clear all NVIC interrupts
	for (uint8_t i = 0; i < 8; i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}

	// STM32G4: remap system memory to 0x00000000
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

	// Re-enable — USB DFU bootloader uses interrupts
	__enable_irq();

	// Read the bootloader's stack pointer and reset handler from system memory
	uint32_t msp          = *(volatile uint32_t*)(SYSTEM_MEMORY_ADDR);
	uint32_t resetHandler = *(volatile uint32_t*)(SYSTEM_MEMORY_ADDR + 4);

	__set_MSP(msp);
	((void (*)(void))resetHandler)();

	while (1);
}

// Call this when you want to trigger DFU (e.g. on USB serial command)
void requestBootloader(void) {
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_RCC_RTC_ENABLE();
	HAL_PWR_EnableBkUpAccess();
	BOOTLOADER_MAGIC_ADDR = BOOTLOADER_MAGIC_VAL;
	NVIC_SystemReset();
}
