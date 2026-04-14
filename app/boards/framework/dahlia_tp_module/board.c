/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Dahlia Touchpad Module board init.
 * Checks PA12 (INT#, connected to BOOT0/PF3) at startup.
 * If high, jumps to the STM32 system bootloader.
 */

#include <zephyr/init.h>
#include <stm32u0xx.h>

#define SYSTEM_MEMORY_BASE 0x1FFF0000U

static int dahlia_tp_module_board_init(void)
{
	/* Enable GPIOA clock */
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	(void)RCC->IOPENR;

	/* Configure PA12 as input */
	GPIOA->MODER &= ~(3U << (12 * 2));

	/* Read PA12 — if high, jump to system bootloader */
	if (GPIOA->IDR & (1U << 12)) {
		__disable_irq();

		RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
		(void)RCC->APBENR2;

		SYSCFG->CFGR1 = (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_MEM_MODE_Msk) | 0x01;

		uint32_t boot_sp = *(volatile uint32_t *)(SYSTEM_MEMORY_BASE);
		uint32_t boot_pc = *(volatile uint32_t *)(SYSTEM_MEMORY_BASE + 4U);

		__set_MSP(boot_sp);
		((void (*)(void))boot_pc)();

		while (1) {}
	}

	return 0;
}

SYS_INIT(dahlia_tp_module_board_init, PRE_KERNEL_1, 0);
