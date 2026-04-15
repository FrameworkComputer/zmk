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

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#include <zmk/event_manager.h>
#include <zmk/events/hid_indicators_changed.h>
#endif

/* Caps Lock LED bit in the HID LED report (HID 1.11 LED page) */
#define HID_LED_CAPS_LOCK   (1U << 1)

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

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
/* Drive the CAPS LED on PC13 from host HID indicator state */
static int dahlia_tp_module_caps_led_listener(const zmk_event_t *eh)
{
	const struct zmk_hid_indicators_changed *ev =
		as_zmk_hid_indicators_changed(eh);
	if (!ev) {
		return 0;
	}

	/* Ensure GPIOC clock and PC13 are configured (failsafe in case
	 * the nboot_sel path didn't run on this boot). */
	RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
	(void)RCC->IOPENR;
	GPIOC->MODER = (GPIOC->MODER & ~(3U << (13 * 2))) | (1U << (13 * 2));

	if (ev->indicators & HID_LED_CAPS_LOCK) {
		GPIOC->BSRR = (1U << 13);
	} else {
		GPIOC->BSRR = (1U << (13 + 16));
	}
	return 0;
}

ZMK_LISTENER(dahlia_tp_module_caps_led, dahlia_tp_module_caps_led_listener);
ZMK_SUBSCRIPTION(dahlia_tp_module_caps_led, zmk_hid_indicators_changed);
#endif
