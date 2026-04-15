/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Dahlia Touchpad Module init.
 *
 * Failsafe: read nBOOT_SEL option bit on every boot. If set (1),
 * clear it and reset the chip. This is a one-time operation on
 * factory-fresh units — after clearing, subsequent boots read
 * nBOOT_SEL=0 and skip the write.
 *
 */

#include <zephyr/init.h>
#include <stm32u0xx.h>

/* indicator */
static void blink_caps_led(int count)
{
	RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
	(void)RCC->IOPENR;
	GPIOC->MODER = (GPIOC->MODER & ~(3U << (13 * 2))) | (1U << (13 * 2));

	for (int i = 0; i < count; i++) {
		GPIOC->BSRR = (1U << 13);
		for (volatile int d = 0; d < 120000; d++) {}
		GPIOC->BSRR = (1U << (13 + 16));
		for (volatile int d = 0; d < 120000; d++) {}
	}
}

static int dahlia_tp_module_nboot_sel_failsafe(void)
{
	/* Already cleared? Nothing to do. */
	if (!(FLASH->OPTR & FLASH_OPTR_nBOOT_SEL))
		return 0;

	/* Blink CAPS LED to signal option byte write is happening */
	blink_caps_led(3);

	/* Wait for flash idle */
	while (FLASH->SR & FLASH_SR_BSY1) {}

	/* Unlock flash */
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;

	/* Unlock option bytes */
	FLASH->OPTKEYR = 0x08192A3B;
	FLASH->OPTKEYR = 0x4C5D6E7F;

	/* Clear nBOOT_SEL (bit 24) in OPTR */
	FLASH->OPTR &= ~FLASH_OPTR_nBOOT_SEL;

	/* Start option byte programming */
	FLASH->CR |= FLASH_CR_OPTSTRT;
	while (FLASH->SR & FLASH_SR_BSY1) {}

	/* Trigger option byte reload — resets the chip */
	FLASH->CR |= FLASH_CR_OBL_LAUNCH;

	/* Should never reach here — rapidly blink LED if OBL_LAUNCH failed */
	while (1) {
		GPIOC->BSRR = (1U << 13);
		for (volatile int d = 0; d < 25000; d++) {}
		GPIOC->BSRR = (1U << (13 + 16));
		for (volatile int d = 0; d < 25000; d++) {}
	}
}

SYS_INIT(dahlia_tp_module_nboot_sel_failsafe, APPLICATION, 99);
