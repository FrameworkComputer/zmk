/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IT8801 register emulation — I2C target side.
 *
 * Emulates the subset of the ITE IT8801 register interface that the EC's
 * stock IT8801 ioexpander driver requires for init and GPIO operations.
 * The emulator is a pure register store; it does NOT drive any physical
 * GPIO itself. Board code registers a callback and decides what each
 * SOVR bit actually does on this board.
 *
 * Registers implemented:
 *   - LBVIDR (0xFE)            : vendor ID low  (0x83)
 *   - HBVIDR (0xFF)            : vendor ID high (0x12)
 *   - SOVR(0..2)  (0x05..0x07) : output value shadow, R/W, callback on
 *                                bit transitions
 *   - IPSR(0..2)  (0x00..0x02) : input pin status, returns 0
 *   - CR(p, n)    (0x0A..0x29) : pin control, writes accepted and ignored,
 *                                reads return 0
 *   - KSO/KSI/GISR/SMBCR/GIECR : accept writes, return 0 on read
 *   - Any other reg            : accept writes, return 0 on read
 */

#define DT_DRV_COMPAT zmk_i2c_target_it8801

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "i2c_it8801_target.h"

LOG_MODULE_REGISTER(i2c_it8801_target, CONFIG_I2C_LOG_LEVEL);

#define REG_IPSR_BASE   0x00  /* 0x00..0x02 */
#define REG_SOVR_BASE   0x05  /* 0x05..0x07 */
#define REG_CR_BASE     0x0A  /* 0x0A..0x29 (3 ports × 8 pins) */
#define REG_CR_END      0x29
#define REG_LBVIDR      0xFE
#define REG_HBVIDR      0xFF

#define IT8801_VENDOR_LO 0x83
#define IT8801_VENDOR_HI 0x12

#define NUM_PORTS 3

struct it8801_target_config {
	struct i2c_dt_spec bus;
	uint8_t sovr_init[NUM_PORTS];
};

struct it8801_target_data {
	struct i2c_target_config i2c_cfg;
	const struct device *dev;
	uint8_t reg_addr;
	bool reg_addr_valid;
	uint8_t sovr[NUM_PORTS];
	it8801_target_change_cb_t change_cb;
};

void it8801_target_set_callback(const struct device *dev,
				it8801_target_change_cb_t cb)
{
	struct it8801_target_data *data = dev->data;
	data->change_cb = cb;
}

uint8_t it8801_target_get_sovr(const struct device *dev, uint8_t port)
{
	struct it8801_target_data *data = dev->data;
	return port < NUM_PORTS ? data->sovr[port] : 0;
}

/* ---------- register read/write ------------------------------------ */

static uint8_t reg_read(const struct device *dev, uint8_t reg)
{
	struct it8801_target_data *data = dev->data;

	if (reg >= REG_SOVR_BASE && reg < REG_SOVR_BASE + NUM_PORTS) {
		return data->sovr[reg - REG_SOVR_BASE];
	}
	if (reg == REG_LBVIDR) {
		return IT8801_VENDOR_LO;
	}
	if (reg == REG_HBVIDR) {
		return IT8801_VENDOR_HI;
	}
	/* IPSR, CR, KSO/KSI/etc, and unhandled regs: return 0. */
	return 0x00;
}

static void reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	struct it8801_target_data *data = dev->data;

	if (reg >= REG_SOVR_BASE && reg < REG_SOVR_BASE + NUM_PORTS) {
		uint8_t port = reg - REG_SOVR_BASE;
		uint8_t old = data->sovr[port];
		uint8_t changed = old ^ val;

		data->sovr[port] = val;

		if (data->change_cb && changed) {
			for (uint8_t pin = 0; pin < 8; pin++) {
				if (changed & BIT(pin)) {
					data->change_cb(dev, port, pin,
							!!(val & BIT(pin)));
				}
			}
		}
		return;
	}
	/* CR and all other registers: accept silently. */
}

/* ---------- I2C target callbacks ----------------------------------- */

static int cb_write_requested(struct i2c_target_config *config)
{
	struct it8801_target_data *data =
		CONTAINER_OF(config, struct it8801_target_data, i2c_cfg);
	data->reg_addr_valid = false;
	return 0;
}

static int cb_write_received(struct i2c_target_config *config, uint8_t val)
{
	struct it8801_target_data *data =
		CONTAINER_OF(config, struct it8801_target_data, i2c_cfg);

	if (!data->reg_addr_valid) {
		data->reg_addr = val;
		data->reg_addr_valid = true;
	} else {
		reg_write(data->dev, data->reg_addr, val);
		data->reg_addr++;
	}
	return 0;
}

static int cb_read_requested(struct i2c_target_config *config, uint8_t *val)
{
	struct it8801_target_data *data =
		CONTAINER_OF(config, struct it8801_target_data, i2c_cfg);

	*val = data->reg_addr_valid ? reg_read(data->dev, data->reg_addr) : 0;
	return 0;
}

static int cb_read_processed(struct i2c_target_config *config, uint8_t *val)
{
	struct it8801_target_data *data =
		CONTAINER_OF(config, struct it8801_target_data, i2c_cfg);

	data->reg_addr++;
	*val = reg_read(data->dev, data->reg_addr);
	return 0;
}

static int cb_stop(struct i2c_target_config *config)
{
	return 0;
}

static const struct i2c_target_callbacks target_callbacks = {
	.write_requested = cb_write_requested,
	.write_received  = cb_write_received,
	.read_requested  = cb_read_requested,
	.read_processed  = cb_read_processed,
	.stop            = cb_stop,
};

/* ---------- init --------------------------------------------------- */

static int it8801_target_init(const struct device *dev)
{
	const struct it8801_target_config *cfg = dev->config;
	struct it8801_target_data *data = dev->data;

	data->dev = dev;
	for (int p = 0; p < NUM_PORTS; p++) {
		data->sovr[p] = cfg->sovr_init[p];
	}

	if (!device_is_ready(cfg->bus.bus)) {
		return -ENODEV;
	}

	data->i2c_cfg.address = cfg->bus.addr;
	data->i2c_cfg.callbacks = &target_callbacks;

	int ret = i2c_target_register(cfg->bus.bus, &data->i2c_cfg);
	if (ret) {
		LOG_ERR("i2c_target_register failed: %d", ret);
		return ret;
	}

	LOG_INF("IT8801 emulation at 0x%02x", cfg->bus.addr);
	return 0;
}

#define IT8801_TARGET_INIT(inst)                                               \
	static struct it8801_target_data it8801_data_##inst;                   \
                                                                               \
	static const struct it8801_target_config it8801_cfg_##inst = {         \
		.bus = I2C_DT_SPEC_INST_GET(inst),                             \
		.sovr_init = {                                                 \
			DT_INST_PROP_OR(inst, sovr0_init, 0xFF),               \
			DT_INST_PROP_OR(inst, sovr1_init, 0xFF),               \
			DT_INST_PROP_OR(inst, sovr2_init, 0xFF),               \
		},                                                             \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(inst,                                            \
			      &it8801_target_init,                             \
			      NULL,                                            \
			      &it8801_data_##inst,                             \
			      &it8801_cfg_##inst,                              \
			      POST_KERNEL,                                     \
			      CONFIG_I2C_TARGET_INIT_PRIORITY,                 \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(IT8801_TARGET_INIT)
