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

#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/kscan.h>
#include <stm32u0xx.h>

#include "../../../module/drivers/i2c/target/i2c_it8801_target.h"
#include <zephyr/drivers/i2c/target/i2c_hid.h>

#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/keymap.h>
#include <zmk/matrix_transform.h>
#include <zmk/hid.h>
#include <dt-bindings/zmk/hid_usage_pages.h>
#include <dt-bindings/zmk/hid_usage.h>

#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#include <zmk/events/hid_indicators_changed.h>
#endif

/* HID modifier byte: bit 2 = Left Alt */
#define HID_MOD_LALT  (1U << 2)
/* FN layer index in the keymap (matches #define FN 1 in ansi-keymap.dtsi) */
#define FN_LAYER_ID   1

/*
 * IT8801 SOVR(2) bit assignments. Port 2 is used because ports 0 and 1
 * have EC-side restrictions on certain pins. Both are Dahlia-specific
 * extensions to the IT8801 emulation spec — the EC writes to SOVR(2)
 * to toggle these.
 */
#define IOEX_IOEX_PORT    2   /* port 2 of SOVR */
#define IOEX_TP_EN_PIN    0   /* port 2, bit 0: touchpad enable (PA15) */
#define IOEX_KB_SCAN_PIN  2   /* port 2, bit 2: keyboard scan enable */

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
static volatile bool dahlia_tp_module_enabled = true;

bool kscan_deghost_input_enabled(void)
{
	return dahlia_tp_module_enabled;
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
	dahlia_tp_module_enabled = on;
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
	if (port != IOEX_IOEX_PORT) {
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

/*
 * Enable I2C1 Fast Mode Plus (20 mA drive) on both peripheral and pad.
 * Zephyr's STM32 I2C driver configures TIMINGR for Fm+ when the DT says
 * I2C_BITRATE_FAST_PLUS, but does NOT touch CR1.FMP or SYSCFG_CFGR1's
 * per-pin Fm+ bits. Without these the rising edges are too slow at 1 MHz
 * and the bus glitches. Runs at APPLICATION/0, after Zephyr's POST_KERNEL
 * I2C init.
 */
static int dahlia_tp_module_enable_i2c_fmp(void)
{
	RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
	(void)RCC->APBENR2;

	SYSCFG->CFGR1 |= SYSCFG_CFGR1_I2C_PB6_FMP |
			 SYSCFG_CFGR1_I2C_PB7_FMP |
			 SYSCFG_CFGR1_I2C_PA9_FMP |
			 SYSCFG_CFGR1_I2C_PA10_FMP;

	/* CR1.FMP is only writable while PE=0. */
	I2C1->CR1 &= ~I2C_CR1_PE;
	I2C1->CR1 |= I2C_CR1_FMP;
	I2C1->CR1 |= I2C_CR1_PE;

	return 0;
}
SYS_INIT(dahlia_tp_module_enable_i2c_fmp, APPLICATION, 0);

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

/* Fn Lock layer. Exists whether or not CONFIG_SETTINGS persists it. */
#define FNLK_LAYER_ID  2

#if IS_ENABLED(CONFIG_SETTINGS)
/* =========================================================================
 * Persistent flags in NVS (wear levelled)
 *
 * Key "dahlia/flags" stores 8 bits.
 *   bit 0 is Fn Lock (mirrors the FNLK layer, id 2)
 *   bits 1..7 are reserved
 * These are restored on boot from NVS.
 * =========================================================================
 */
#define DAHLIA_FLAG_FNLK   (1U << 0)

static uint8_t dahlia_tp_module_flags;
static bool dahlia_tp_module_flags_loaded;

static int dahlia_flags_settings_set(const char *key, size_t len,
				     settings_read_cb read_cb, void *cb_arg)
{
	const char *next;
	if (settings_name_steq(key, "flags", &next) && !next) {
		uint8_t val = 0;
		if (read_cb(cb_arg, &val, sizeof(val)) > 0) {
			dahlia_tp_module_flags = val;
			if (dahlia_tp_module_flags & DAHLIA_FLAG_FNLK) {
				zmk_keymap_layer_activate(FNLK_LAYER_ID, false);
			}
		}
		return 0;
	}
	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(dahlia_flags, "dahlia", NULL,
			       dahlia_flags_settings_set, NULL, NULL);

/* Scheduled after ZMK keymap init */
static int dahlia_tp_module_flags_init(void)
{
	int err = settings_subsys_init();
	if (err) {
		return err;
	}
	err = settings_load_subtree("dahlia");
	dahlia_tp_module_flags_loaded = true;
	return err;
}
SYS_INIT(dahlia_tp_module_flags_init, APPLICATION, 95);

/* Update one bit of the flags byte; if it changed and the initial load
 * has finished, persist the whole byte. */
static void dahlia_tp_module_flags_update(uint8_t mask, bool value)
{
	uint8_t prev = dahlia_tp_module_flags;
	if (value) {
		dahlia_tp_module_flags |= mask;
	} else {
		dahlia_tp_module_flags &= ~mask;
	}
	if (dahlia_tp_module_flags != prev && dahlia_tp_module_flags_loaded) {
		(void)settings_save_one("dahlia/flags", &dahlia_tp_module_flags,
					sizeof(dahlia_tp_module_flags));
	}
}

static int dahlia_tp_module_fnlk_layer_listener(const zmk_event_t *eh)
{
	const struct zmk_layer_state_changed *ev =
		as_zmk_layer_state_changed(eh);
	if (ev && ev->layer == FNLK_LAYER_ID) {
		dahlia_tp_module_flags_update(DAHLIA_FLAG_FNLK, ev->state);
	}
	return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(dahlia_tp_module_fnlk_layer, dahlia_tp_module_fnlk_layer_listener);
ZMK_SUBSCRIPTION(dahlia_tp_module_fnlk_layer, zmk_layer_state_changed);
#endif /* CONFIG_SETTINGS */

/* =========================================================================
 * Fn / LCtrl swap
 *
 * Swaps the two keys' bindings in the RAM keymap on all layers, using
 * absolute values captured at first use (idempotent). RAM-only: never
 * saved, so cold boot restores the stock keymap — keep it that way.
 * =========================================================================
 */

#define DAHLIA_SWAP_LCTRL_ROW 1
#define DAHLIA_SWAP_LCTRL_COL 14
#define DAHLIA_SWAP_FN_ROW    0
#define DAHLIA_SWAP_FN_COL    16

ZMK_MATRIX_TRANSFORM_EXTERN(DT_CHOSEN(zmk_matrix_transform));
#define DAHLIA_TRANSFORM  ZMK_MATRIX_TRANSFORM_T_FOR_NODE(DT_CHOSEN(zmk_matrix_transform))

/* Boot bindings of the two positions, captured per layer before the first
 * swap ever runs. ~100 bytes total. */
static struct zmk_behavior_binding dahlia_boot_lctrl[ZMK_KEYMAP_LAYERS_LEN];
static struct zmk_behavior_binding dahlia_boot_fn[ZMK_KEYMAP_LAYERS_LEN];
static bool dahlia_boot_snapshot_done;
static bool dahlia_swap_state;

/* Returns 0 on success, negative on lookup/snapshot failure. */
static int dahlia_swap_snapshot(int32_t *pos_lctrl, int32_t *pos_fn)
{
	*pos_lctrl = zmk_matrix_transform_row_column_to_position(
		DAHLIA_TRANSFORM, DAHLIA_SWAP_LCTRL_ROW, DAHLIA_SWAP_LCTRL_COL);
	*pos_fn = zmk_matrix_transform_row_column_to_position(
		DAHLIA_TRANSFORM, DAHLIA_SWAP_FN_ROW, DAHLIA_SWAP_FN_COL);
	if (*pos_lctrl < 0 || *pos_fn < 0)
		return -EINVAL;

	if (dahlia_boot_snapshot_done)
		return 0;

	for (zmk_keymap_layer_id_t l = 0; l < ZMK_KEYMAP_LAYERS_LEN; l++) {
		const struct zmk_behavior_binding *a =
			zmk_keymap_get_layer_binding_at_idx(l, *pos_lctrl);
		const struct zmk_behavior_binding *b =
			zmk_keymap_get_layer_binding_at_idx(l, *pos_fn);
		if (!a || !b)
			return -EINVAL;
		dahlia_boot_lctrl[l] = *a;
		dahlia_boot_fn[l] = *b;
	}
	dahlia_boot_snapshot_done = true;
	return 0;
}

static int dahlia_set_swap(bool swapped)
{
	int32_t pos_lctrl, pos_fn;
	int ret = dahlia_swap_snapshot(&pos_lctrl, &pos_fn);

	if (ret < 0)
		return ret;

	for (zmk_keymap_layer_id_t l = 0; l < ZMK_KEYMAP_LAYERS_LEN; l++) {
		ret = zmk_keymap_set_layer_binding_at_idx(l, pos_lctrl,
			swapped ? dahlia_boot_fn[l] : dahlia_boot_lctrl[l]);
		if (ret < 0)
			return ret;
		ret = zmk_keymap_set_layer_binding_at_idx(l, pos_fn,
			swapped ? dahlia_boot_lctrl[l] : dahlia_boot_fn[l]);
		if (ret < 0)
			return ret;
	}

	dahlia_swap_state = swapped;
	return 0;
}

/* =========================================================================
 * Misc flags, and update protection
 *
 * Slot 0 of the misc-flags partition is update protect: the BIOS replaces
 * this firmware when the version it bundles is newer, and a deck can ask
 * not to be replaced. The request travels with the deck, so it is
 * reported to the host in the HID descriptor's reserved field (offset 26)
 * as DAHLIA_TP_MODULE_FW_FLAG_NO_AUTO_UPDATE. Set it at build time with
 * fw-flags = <0x01> on the i2c_hid node, or, to protect a stock image
 * without rebuilding it, have the flashing tool set the slot; this file
 * contributes the latter. Slot encoding is in the board dtsi.
 *
 * Not a write protect: EC_CMD_KBD_FLASH_STM32 never consults it and a
 * mass erase clears it. It only asks an automatic updater to stay away.
 *
 * A slot that is neither all ones nor all zeros counts as clear, so a
 * half-written slot cannot block a recovery update. Flash here is ECC'd
 * per doubleword, so an interrupted write could fault on read — same
 * exposure as an interrupted image write, and the same outcome: a deck
 * that cannot answer, which the BIOS reflashes, clearing the slot.
 * =========================================================================
 */

/* Bit in the HID descriptor's reserved field. */
#define DAHLIA_TP_MODULE_FW_FLAG_NO_AUTO_UPDATE 0x01

/* Slot numbers in the misc-flags area. 8 slots, 0..7. */
#define DAHLIA_TP_MODULE_MISC_SLOT_NO_AUTO_UPDATE 0

#define DAHLIA_TP_MODULE_MISC_FLAGS_ADDR \
	(DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) + \
	 DT_REG_ADDR(DT_NODELABEL(misc_flags_partition)))

#define DAHLIA_TP_MODULE_MISC_SLOT_SIZE 8

BUILD_ASSERT(DT_REG_SIZE(DT_NODELABEL(misc_flags_partition)) >=
		     (DAHLIA_TP_MODULE_MISC_SLOT_NO_AUTO_UPDATE + 1) *
			     DAHLIA_TP_MODULE_MISC_SLOT_SIZE,
	     "misc-flags partition too small for the slots used here");

static bool dahlia_tp_module_misc_slot_set(unsigned int slot)
{
	const volatile uint32_t *p =
		(const volatile uint32_t *)(DAHLIA_TP_MODULE_MISC_FLAGS_ADDR +
					    slot * DAHLIA_TP_MODULE_MISC_SLOT_SIZE);

	return p[0] == 0 && p[1] == 0;
}

/*
 * Called from the i2c-hid driver's init, before the I2C target is
 * registered, so it must not touch anything that initialises later. This
 * only reads memory-mapped flash, which is always available.
 */
uint32_t i2c_hid_target_board_fw_flags(void)
{
	uint32_t flags = 0;

	if (dahlia_tp_module_misc_slot_set(DAHLIA_TP_MODULE_MISC_SLOT_NO_AUTO_UPDATE)) {
		flags |= DAHLIA_TP_MODULE_FW_FLAG_NO_AUTO_UPDATE;
	}

	return flags;
}

#if IS_ENABLED(CONFIG_ZMK_HID_FRAMEWORK_VENDOR_COMMAND)
/* =========================================================================
 * Vendor command channel (Feature report ID 0x20).
 *
 * Command:  SET_REPORT  [opcode][args…]
 * Response: GET_REPORT  [opcode][status][data…]  (last response is cached)
 *
 * Opcodes and payloads: see VENDOR_PROTOCOL.md.
 * =========================================================================
 */

/* Version source of truth is the DT version-id (= HID wVersionID),
 * nibble-encoded: 0x1234 → 12.3.4. Hash/dirty come from the header
 * generated by git_version.cmake. */
#include "dahlia_git_version.h"
#define DAHLIA_FW_VERSION  DT_PROP(DT_NODELABEL(i2c_hid), version_id)
static const uint8_t dahlia_fw_hash[8] = DAHLIA_GIT_HASH_BYTES;

/* Response buffer, populated on each SET_REPORT, drained by next GET_REPORT.
 * Max response fits in 32-byte report: 1 byte ID (added by driver) +
 * opcode + status + up to 29 bytes data. */
static uint8_t dahlia_vendor_response[31];
static uint16_t dahlia_vendor_response_len;

static void dahlia_vendor_reply(uint8_t opcode, uint8_t status,
				const uint8_t *data, size_t data_len)
{
	if (data_len > sizeof(dahlia_vendor_response) - 2) {
		data_len = sizeof(dahlia_vendor_response) - 2;
	}
	dahlia_vendor_response[0] = opcode;
	dahlia_vendor_response[1] = status;
	if (data && data_len) {
		memcpy(&dahlia_vendor_response[2], data, data_len);
	}
	dahlia_vendor_response_len = 2 + data_len;
}

#define STM32_FLASHSIZE_ADDR   0x1FFF6EA0u  /* 16-bit KB value */
#define STM32_UID_ADDR         0x1FFF6E50u  /* 12 bytes (96 bits) */

static uint16_t dahlia_tp_module_read_chip_id(void) {
	bool dbg_was_on = (RCC->DBGCFGR & RCC_DBGCFGR_DBGEN) != 0U;
	uint16_t dev_id;

	RCC->DBGCFGR |= RCC_DBGCFGR_DBGEN;
	(void)RCC->DBGCFGR;

	dev_id = (uint16_t)(DBGMCU->IDCODE & DBGMCU_IDCODE_DEV_ID);

	if (!dbg_was_on) {
		RCC->DBGCFGR &= ~RCC_DBGCFGR_DBGEN;
	}

	return dev_id;
}

static void dahlia_vendor_handle_get_hw_info(void)
{
	uint16_t chip_id = dahlia_tp_module_read_chip_id();
	uint16_t flash_kb = *(volatile uint16_t *)STM32_FLASHSIZE_ADDR;
	uint16_t ram_kb = 40;  /* STM32U073: SRAM1 32K + SRAM2 8K */

	uint8_t payload[2 + 2 + 2 + 12];
	payload[0] = chip_id & 0xFF;
	payload[1] = (chip_id >> 8) & 0xFF;
	payload[2] = flash_kb & 0xFF;
	payload[3] = (flash_kb >> 8) & 0xFF;
	payload[4] = ram_kb & 0xFF;
	payload[5] = (ram_kb >> 8) & 0xFF;
	memcpy(&payload[6], (const void *)STM32_UID_ADDR, 12);

	dahlia_vendor_reply(0x02, 0x00, payload, sizeof(payload));
}

static void dahlia_vendor_handle_get_fw_version(void)
{
	uint8_t payload[3 + sizeof(dahlia_fw_hash)];
	payload[0] = DAHLIA_FW_VERSION & 0xFF;
	payload[1] = (DAHLIA_FW_VERSION >> 8) & 0xFF;
	payload[2] = DAHLIA_GIT_DIRTY;
	memcpy(&payload[3], dahlia_fw_hash, sizeof(dahlia_fw_hash));

	dahlia_vendor_reply(0x01, 0x00, payload, sizeof(payload));
}

/* ---- State flags (opcodes 0x12 / 0x13) -----------------------------
 *
 * One bitmask over the deck's reportable state. SET takes a mask as well
 * as a value so a caller only touches the bits it names: the bits do not
 * share capabilities — update protect is read-only — and a whole-word
 * write would force a read-modify-write, which races with the vendor
 * writes the EC makes of its own accord.
 *
 * Booleans only, deliberately. If a setting ever needs a wider value it
 * should get its own opcode rather than stretching this one.
 */
#define DAHLIA_TP_MODULE_VFLAG_FN_LOCK (1U << 0) /* R/W, persisted in NVS */
#define DAHLIA_TP_MODULE_VFLAG_FN_CTRL_SWAP (1U << 1) /* R/W, RAM only; = opcode 0x10 */
#define DAHLIA_TP_MODULE_VFLAG_UPDATE_PROTECT (1U << 2) /* R/O, misc-flags slot 0 */

/*
 * GET_FLAGS reports three masks, so a host can work out what this build
 * supports without keeping a table of firmware versions:
 *
 *   present  - the bits this build implements
 *   flags    - their values, meaningful only within present
 *   writable - the subset of present that SET_FLAGS will accept
 *
 * Without present, a bit that exists and reads 0 is indistinguishable
 * from one that does not exist at all.
 *
 * Assignments are append-only: a retired setting drops out of present,
 * and its bit is never reused. Reusing one would leave a host that knows
 * the old meaning silently misreading the new one, which is worse than
 * it not knowing the bit at all.
 */
#define DAHLIA_TP_MODULE_VFLAGS_PRESENT \
	(DAHLIA_TP_MODULE_VFLAG_FN_LOCK | DAHLIA_TP_MODULE_VFLAG_FN_CTRL_SWAP | \
	 DAHLIA_TP_MODULE_VFLAG_UPDATE_PROTECT)

/* The writable bits are the low ones, so this stays a contiguous mask. */
#define DAHLIA_TP_MODULE_VFLAGS_WRITABLE \
	(DAHLIA_TP_MODULE_VFLAG_FN_LOCK | DAHLIA_TP_MODULE_VFLAG_FN_CTRL_SWAP)

static uint32_t dahlia_tp_module_vflags_get(void)
{
	uint32_t flags = 0;

	if (zmk_keymap_layer_active(FNLK_LAYER_ID)) {
		flags |= DAHLIA_TP_MODULE_VFLAG_FN_LOCK;
	}
	if (dahlia_tp_module_misc_slot_set(DAHLIA_TP_MODULE_MISC_SLOT_NO_AUTO_UPDATE)) {
		flags |= DAHLIA_TP_MODULE_VFLAG_UPDATE_PROTECT;
	}
	if (dahlia_swap_state) {
		flags |= DAHLIA_TP_MODULE_VFLAG_FN_CTRL_SWAP;
	}

	return flags;
}

/* Returns a vendor status byte: 0 ok, 1 bad argument, 2 internal error. */
static uint8_t dahlia_tp_module_vflags_set(uint32_t mask, uint32_t value)
{
	if (mask & ~DAHLIA_TP_MODULE_VFLAGS_WRITABLE) {
		return 0x01; /* unknown or read-only bit */
	}

	if (mask & DAHLIA_TP_MODULE_VFLAG_FN_LOCK) {
		bool want = (value & DAHLIA_TP_MODULE_VFLAG_FN_LOCK) != 0;

		/* The FNLK layer is the source of truth; the layer-change
		 * listener mirrors it into NVS. Skipping the no-op case keeps
		 * a redundant set from costing a flash write.
		 *
		 * Locking, to match the keymap's own &tog FNLK: ZMK ignores a
		 * non-locking deactivate of a locked layer, and returns 0
		 * when it does, so asking politely here would silently fail.
		 */
		if (want != zmk_keymap_layer_active(FNLK_LAYER_ID)) {
			int rv = want ? zmk_keymap_layer_activate(FNLK_LAYER_ID,
								 true)
				      : zmk_keymap_layer_deactivate(FNLK_LAYER_ID,
								    true);
			if (rv < 0) {
				return 0x02;
			}
		}
	}

	if (mask & DAHLIA_TP_MODULE_VFLAG_FN_CTRL_SWAP) {
		bool want = (value & DAHLIA_TP_MODULE_VFLAG_FN_CTRL_SWAP) != 0;

		if (dahlia_set_swap(want) < 0) {
			return 0x02;
		}
	}

	/* Confirm rather than trust: the layer API declines some requests
	 * without saying so in its return value.
	 */
	if ((dahlia_tp_module_vflags_get() & mask) != (value & mask)) {
		return 0x02;
	}

	return 0x00;
}

void zmk_i2c_hid_vendor_set_report(uint8_t id, const uint8_t *data, uint16_t len)
{
	/* Some HID-over-I2C implementations prefix the payload with the
	 * report-ID byte (per HID class spec), others don't. Tolerate both:
	 * skip a leading byte equal to our report ID (0x20) if present, then
	 * treat data[0] as the opcode. Since no valid opcode is 0x20 (the
	 * config ops use 0x10/0x11), there's no ambiguity. */
	if (id != 0x20 || len < 1) {
		return;
	}
	if (data[0] == 0x20 && len >= 2) {
		data++;
		len--;
	}
	uint8_t opcode = data[0];
	const uint8_t *args = data + 1;
	uint16_t args_len = len - 1;

	switch (opcode) {
	case 0x01: /* GET_FW_VERSION */
		dahlia_vendor_handle_get_fw_version();
		break;

	case 0x02: /* GET_HW_INFO */
		dahlia_vendor_handle_get_hw_info();
		break;

	case 0x10: /* SET_SWAP_FN_LCTRL */
		if (args_len < 1) {
			dahlia_vendor_reply(opcode, 0x01, NULL, 0);  /* EINVAL */
			break;
		}
		if (dahlia_set_swap(args && args[0]) < 0) {
			dahlia_vendor_reply(opcode, 0x02, NULL, 0);  /* internal error */
			break;
		}
		dahlia_vendor_reply(opcode, 0x00, NULL, 0);
		break;

	case 0x11: { /* GET_SWAP_FN_LCTRL */
		uint8_t v = dahlia_swap_state ? 1 : 0;
		dahlia_vendor_reply(opcode, 0x00, &v, 1);
		break;
	}

	case 0x12: { /* SET_FLAGS */
		uint8_t payload[4];
		uint8_t status;

		if (args_len < 8) {
			dahlia_vendor_reply(opcode, 0x01, NULL, 0);  /* EINVAL */
			break;
		}
		status = dahlia_tp_module_vflags_set(sys_get_le32(&args[0]),
						     sys_get_le32(&args[4]));
		/* Always report the resulting state, so a caller needs no
		 * follow-up GET even when the set was rejected. */
		sys_put_le32(dahlia_tp_module_vflags_get(), payload);
		dahlia_vendor_reply(opcode, status, payload, sizeof(payload));
		break;
	}

	case 0x13: { /* GET_FLAGS */
		uint8_t payload[12];

		sys_put_le32(DAHLIA_TP_MODULE_VFLAGS_PRESENT, &payload[0]);
		sys_put_le32(dahlia_tp_module_vflags_get(), &payload[4]);
		sys_put_le32(DAHLIA_TP_MODULE_VFLAGS_WRITABLE, &payload[8]);
		dahlia_vendor_reply(opcode, 0x00, payload, sizeof(payload));
		break;
	}

	default:
		dahlia_vendor_reply(opcode, 0xFF, NULL, 0);  /* Unknown opcode */
		break;
	}
}

int zmk_i2c_hid_vendor_get_report(uint8_t id, uint8_t *buf, uint16_t max_len)
{
	if (id != 0x20) {
		return 0;
	}
	uint16_t n = dahlia_vendor_response_len;
	if (n > max_len) n = max_len;
	if (n) {
		memcpy(buf, dahlia_vendor_response, n);
	}
	return n;
}

/* Seed the response buffer so a bare GET_REPORT returns the fw version. */
static int dahlia_vendor_seed_response(void)
{
	dahlia_vendor_handle_get_fw_version();
	return 0;
}
SYS_INIT(dahlia_vendor_seed_response, APPLICATION, 92);
#endif /* CONFIG_ZMK_HID_FRAMEWORK_VENDOR_COMMAND */
