/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Dahlia Touchpad Module init.
 *
 * Failsafe: read nBOOT_SEL option bit on every boot. If set,
 * clear it and reset the chip. This is a one-time operation on
 * fresh units — after clearing, subsequent boots should read
 * nBOOT_SEL=0 and skip the write.
 *
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/kscan.h>
#include <stm32u0xx.h>

#include "../../../module/drivers/i2c/target/i2c_it8801_target.h"

#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/keymap.h>
#include <zmk/hid.h>
#include <dt-bindings/zmk/hid_usage_pages.h>
#include <dt-bindings/zmk/hid_usage.h>

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#include <zmk/events/hid_indicators_changed.h>
#endif

/* HID modifier byte: bit 2 = Left Alt */
#define HID_MOD_LALT  (1U << 2)
/* FN layer index in the keymap (matches #define FN 1 in ansi-keymap.dtsi) */
#define FN_LAYER_ID   1

/*
 * IT8801 SOVR(0) bit assignments. Bit 0 = TP_EN matches the dahlia
 * IT8801 emulation spec (dahlia-it8801-emulation.md) so the EC's stock
 * IT8801 driver works without modification. Bit 1 (kb scan enable) is
 * a Dahlia-specific extension — the EC must opt in to use it.
 */
#define IOEX_TP_EN_PIN    0   /* port 0, bit 0: touchpad enable (PA15) — spec */
#define IOEX_KB_SCAN_PIN  1   /* port 0, bit 1: keyboard scan enable — extension */

/* Caps Lock LED bit in the HID LED report (HID 1.11 LED page) */
#define HID_LED_CAPS_LOCK   (1U << 1)

/*
 * Capture fault information, in TAMP backup registers so they are available
 * to read after a hot reset using SWD.
 *
 * Layout (BKP0R..BKP8R, 9 x 32-bit):
 *   BKP0R: magic (0xDEADBEEF when captured; anything else = no fault)
 *   BKP1R: Zephyr fatal reason code (see zephyr/include/zephyr/fatal_types.h)
 *   BKP2R: PC at fault
 *   BKP3R: LR at fault
 *   BKP4R: xPSR at fault
 *   BKP5R: R0
 *   BKP6R: R3
 *   BKP7R: R12 (IP)
 *   BKP8R: SCB->ICSR (VECTACTIVE lower 9 bits = which exception fired;
 *                     HardFault = 3 on Cortex-M0+)
 *
 * On next boot, if BKP0R holds the magic, we start a blink thread that
 * flashes the CAPS LED in a 3-short + 1-long pattern.
 */

#define DAHLIA_FAULT_MAGIC   0xDEADBEEFu

/* indicator — ~200 ms on, ~200 ms off per cycle at 48 MHz */
static void blink_caps_led(int count)
{
	RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
	(void)RCC->IOPENR;
	GPIOC->MODER = (GPIOC->MODER & ~(3U << (13 * 2))) | (1U << (13 * 2));

	for (int i = 0; i < count; i++) {
		GPIOC->BSRR = (1U << 13);
		for (volatile int d = 0; d < 20000; d++) {}
		GPIOC->BSRR = (1U << (13 + 16));
		for (volatile int d = 0; d < 20000; d++) {}
	}
}

/* Explicitly prevent Stop mode — clear SLEEPDEEP in SCB SCR.
 * With SLEEPDEEP=0, it keeps I2C and GPIO responsive.
 */
static int dahlia_tp_module_prevent_stop_mode(void)
{
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	return 0;
}
SYS_INIT(dahlia_tp_module_prevent_stop_mode, PRE_KERNEL_1, 0);

static int dahlia_tp_module_nboot_sel_failsafe(void)
{
	/* Already cleared? Nothing to do. */
	if (!(FLASH->OPTR & FLASH_OPTR_nBOOT_SEL))
		return 0;

	/* Blink CAPS LED to signal option byte write is happening */
	blink_caps_led(3);

	/* Disable interrupts — the unlock sequences require two
	 * back-to-back writes with nothing in between. */
	unsigned int key = irq_lock();

	/* Wait for flash idle */
	while (FLASH->SR & FLASH_SR_BSY1) {}

	/* Clear any stale error flags */
	FLASH->SR = FLASH->SR;

	/* Unlock flash */
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;

	if (FLASH->CR & FLASH_CR_LOCK) {
		/* Unlock failed — blink 5 short */
		irq_unlock(key);
		blink_caps_led(5);
		return -1;
	}

	/* Unlock option bytes */
	FLASH->OPTKEYR = 0x08192A3B;
	FLASH->OPTKEYR = 0x4C5D6E7F;

	if (FLASH->CR & FLASH_CR_OPTLOCK) {
		/* OB unlock failed — blink 7 short */
		irq_unlock(key);
		blink_caps_led(7);
		return -1;
	}

	/* Clear nBOOT_SEL (bit 24) in OPTR */
	FLASH->OPTR &= ~FLASH_OPTR_nBOOT_SEL;

	/* Start option byte programming */
	SET_BIT(FLASH->CR, FLASH_CR_OPTSTRT);
	while (FLASH->SR & FLASH_SR_BSY1) {}
	CLEAR_BIT(FLASH->CR, FLASH_CR_OPTSTRT);

	/* Re-lock flash */
	SET_BIT(FLASH->CR, FLASH_CR_LOCK);

	irq_unlock(key);

	/* The hardware registers are updated on next boot */
	return 0;
}

SYS_INIT(dahlia_tp_module_nboot_sel_failsafe, APPLICATION, 99);

/* Non-zero while a captured fault is being signaled on the LED. */
static atomic_t dahlia_tp_module_fault_present = ATOMIC_INIT(0);

/* Enable backup-domain write protection, required by TAMP backup registers.*/
static void dahlia_tp_module_backup_enable_access(void)
{
	RCC->APBENR1 |= RCC_APBENR1_PWREN | RCC_APBENR1_RTCAPBEN;
	(void)RCC->APBENR1;
	PWR->CR1 |= PWR_CR1_DBP;
	(void)PWR->CR1;
}

/* Strong override of Zephyr's weak fatal error handler. Runs in handler mode
 * (IRQs disabled below). We only touch registers — no stack allocation, no
 * library calls — to minimize risk of faulting again inside the handler. */
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	__disable_irq();
	dahlia_tp_module_backup_enable_access();

	uint32_t pc = 0, lr = 0, xpsr = 0, r0 = 0, r3 = 0, r12 = 0;
	if (esf) {
		pc = esf->basic.pc;
		lr = esf->basic.lr;
		xpsr = esf->basic.xpsr;
		r0 = esf->basic.r0;
		r3 = esf->basic.r3;
		r12 = esf->basic.r12;
	}

	TAMP->BKP1R = reason;
	TAMP->BKP2R = pc;
	TAMP->BKP3R = lr;
	TAMP->BKP4R = xpsr;
	TAMP->BKP5R = r0;
	TAMP->BKP6R = r3;
	TAMP->BKP7R = r12;
	TAMP->BKP8R = SCB->ICSR;
	/* Write magic last — without it the magic is ignored on next boot. */
	TAMP->BKP0R = DAHLIA_FAULT_MAGIC;

	__DSB();
	NVIC_SystemReset();
	for (;;) {}
}

/* Blink pattern: 3 short pulses, pause, 1 long pulse, pause */
static void dahlia_tp_module_fault_blink_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
	(void)RCC->IOPENR;
	GPIOC->MODER = (GPIOC->MODER & ~(3U << (13 * 2))) | (1U << (13 * 2));

	while (atomic_get(&dahlia_tp_module_fault_present)) {
		for (int i = 0; i < 3; i++) {
			GPIOC->BSRR = (1U << 13);
			k_msleep(120);
			GPIOC->BSRR = (1U << (13 + 16));
			k_msleep(120);
		}
		k_msleep(300);
		GPIOC->BSRR = (1U << 13);
		k_msleep(600);
		GPIOC->BSRR = (1U << (13 + 16));
		k_msleep(900);
	}
}

K_THREAD_STACK_DEFINE(dahlia_tp_module_fault_blink_stack, 512);
static struct k_thread dahlia_tp_module_fault_blink_tcb;

/* Check backup regs on boot for captured fault, and kick off the blink thread
 * if one is present. Runs at APPLICATION/50 so the kernel is fully up and
 * k_msleep works from the spawned thread. */
static int dahlia_tp_module_fault_check(void) {
	dahlia_tp_module_backup_enable_access();
	if (TAMP->BKP0R == DAHLIA_FAULT_MAGIC) {
		atomic_set(&dahlia_tp_module_fault_present, 1);
		k_thread_create(&dahlia_tp_module_fault_blink_tcb,
				dahlia_tp_module_fault_blink_stack,
				K_THREAD_STACK_SIZEOF(dahlia_tp_module_fault_blink_stack),
				dahlia_tp_module_fault_blink_fn,
				NULL, NULL, NULL,
				K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
	}
	return 0;
}
SYS_INIT(dahlia_tp_module_fault_check, APPLICATION, 50);

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
/* Drive the CAPS LED on PC13 from host HID indicator state */
static int dahlia_tp_module_caps_led_listener(const zmk_event_t *eh)
{
	/* Fault blink owns the LED — don't fight it. */
	if (atomic_get(&dahlia_tp_module_fault_present)) {
		return 0;
	}

	const struct zmk_hid_indicators_changed *ev =
		as_zmk_hid_indicators_changed(eh);
	if (!ev) {
		return 0;
	}

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

static bool dahlia_tp_en_state = true;

/*
 * Single source of truth for "is the input deck enabled".
 *   true  → keyboard scans + touchpad enabled (PA15 high)
 *   false → keyboard scans dropped + touchpad disabled (PA15 low)
 *
 * The deghost driver calls the weak hook below before forwarding any
 * matrix event; we override it to return this flag.
 */
static volatile bool dahlia_inputdeck_enabled = true;

bool kscan_deghost_input_enabled(void)
{
	return dahlia_inputdeck_enabled;
}

static void dahlia_tp_module_set_tp_en(bool on)
{
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	(void)RCC->IOPENR;

	/* PA15: output, no pulls */
	GPIOA->PUPDR &= ~(3U << (15 * 2));
	GPIOA->MODER = (GPIOA->MODER & ~(3U << (15 * 2))) | (1U << (15 * 2));
	GPIOA->BSRR = on ? (1U << 15) : (1U << (15 + 16));

	dahlia_tp_en_state = on;
}

static void dahlia_inputdeck_set(bool on)
{
	dahlia_inputdeck_enabled = on;
}

static int dahlia_tp_module_force_tp_en_high(void)
{
	dahlia_tp_module_set_tp_en(true);
	return 0;
}
SYS_INIT(dahlia_tp_module_force_tp_en_high, PRE_KERNEL_1, 0);

/* Fn + LAlt + T: toggle TP_EN only (does not affect keyboard scanning). */
static int dahlia_tp_module_shortcut_listener(const zmk_event_t *eh)
{
	const struct zmk_keycode_state_changed *ev =
		as_zmk_keycode_state_changed(eh);
	if (!ev) {
		return 0;
	}
	if (!ev->state ||
	    ev->usage_page != HID_USAGE_KEY ||
	    ev->keycode != HID_USAGE_KEY_KEYBOARD_T) {
		return 0;
	}
	if (!zmk_keymap_layer_active(FN_LAYER_ID)) {
		return 0;
	}
	if (!(zmk_hid_get_explicit_mods() & HID_MOD_LALT)) {
		return 0;
	}

	dahlia_tp_module_set_tp_en(!dahlia_tp_en_state);
	return 0;
}

ZMK_LISTENER(dahlia_tp_module_shortcut, dahlia_tp_module_shortcut_listener);
ZMK_SUBSCRIPTION(dahlia_tp_module_shortcut, zmk_keycode_state_changed);

/*
 * EC writes to SOVR(0). Each bit has its own meaning:
 *   bit 0 → keyboard scan enable (gates deghost forwarding)
 *   bit 1 → touchpad enable      (drives PA15)
 * Other bits and other ports are ignored.
 */
static void dahlia_tp_module_on_ioex_change(const struct device *dev,
					    uint8_t port, uint8_t pin,
					    bool level)
{
	ARG_UNUSED(dev);
	if (port != 0) {
		return;
	}
	switch (pin) {
	case IOEX_KB_SCAN_PIN: {
		dahlia_inputdeck_set(level);

		/* Also stop the matrix polling so we don't burn CPU/GPIO power
		 * while disabled. The weak hook in deghost is still in place
		 * as a safety net for any in-flight event. */
		const struct device *kscan =
			DEVICE_DT_GET(DT_CHOSEN(zmk_kscan));
		if (device_is_ready(kscan)) {
			if (level) {
				kscan_enable_callback(kscan);
			} else {
				kscan_disable_callback(kscan);
			}
		}
		break;
	}
	case IOEX_TP_EN_PIN:
		dahlia_tp_module_set_tp_en(level);
		break;
	default:
		break;
	}
}

static int dahlia_tp_module_register_ioex_cb(void)
{
	const struct device *ioex = DEVICE_DT_GET(DT_NODELABEL(i2c_ioex));
	if (!device_is_ready(ioex)) {
		return -ENODEV;
	}
	it8801_target_set_callback(ioex, dahlia_tp_module_on_ioex_change);
	return 0;
}
SYS_INIT(dahlia_tp_module_register_ioex_cb, APPLICATION, 90);
