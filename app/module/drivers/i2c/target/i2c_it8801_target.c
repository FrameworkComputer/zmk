/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IT8801 GPIO expander emulation — I2C target side.
 *
 * Emulates a minimal subset of the ITE IT8801 register map so a host
 * EC running the standard IT8801 ioexpander driver can read/write
 * GPIOs on this device.
 *
 * Only port 0 is implemented.  The register set covers:
 *   - IPSR(0) at 0x00  — read pin states
 *   - SOVR(0) at 0x05  — read/write output values
 *   - CR(0,x) at 0x0A‥0x11 — per-pin control (direction, flags)
 *   - LBVIDR  at 0xFE  — vendor ID low  (0x83)
 *   - HBVIDR  at 0xFF  — vendor ID high (0x12)
 */

#define DT_DRV_COMPAT zmk_i2c_target_it8801

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_it8801_target, CONFIG_I2C_LOG_LEVEL);

/* IT8801 register addresses (port 0 only) */
#define REG_IPSR0        0x00   /* input pin status */
#define REG_SOVR0        0x05   /* set output value */
#define REG_CR_BASE      0x0A   /* control regs: 0x0A + bit_pos */
#define REG_CR_END       0x11   /* 0x0A + 7 */
#define REG_LBVIDR       0xFE   /* vendor ID low */
#define REG_HBVIDR       0xFF   /* vendor ID high */

/* Vendor ID that the EC driver checks */
#define IT8801_VENDOR_LO 0x83
#define IT8801_VENDOR_HI 0x12

/* CR bit fields */
#define CR_GPIODIR       BIT(5) /* 1 = output */

#define MAX_GPIOS 8

struct it8801_target_config {
	struct i2c_dt_spec bus;
	const struct gpio_dt_spec *gpios;
	uint8_t ngpios;
};

struct it8801_target_data {
	struct i2c_target_config i2c_cfg;
	const struct device *dev;
	uint8_t reg_addr;      /* register address from first write byte */
	bool reg_addr_valid;
	uint8_t sovr;          /* shadow output register */
	uint8_t cr[MAX_GPIOS]; /* shadow control registers */
};

/* ---------- GPIO helpers ------------------------------------------- */

static void apply_sovr(const struct device *dev)
{
	const struct it8801_target_config *cfg = dev->config;
	struct it8801_target_data *data = dev->data;

	for (int i = 0; i < cfg->ngpios; i++) {
		if (data->cr[i] & CR_GPIODIR) {
			gpio_pin_set_dt(&cfg->gpios[i],
					(data->sovr >> i) & 1);
		}
	}
}

static uint8_t read_ipsr(const struct device *dev)
{
	const struct it8801_target_config *cfg = dev->config;
	uint8_t val = 0;

	for (int i = 0; i < cfg->ngpios; i++) {
		if (gpio_pin_get_dt(&cfg->gpios[i])) {
			val |= BIT(i);
		}
	}
	return val;
}

static uint8_t reg_read(const struct device *dev, uint8_t reg)
{
	struct it8801_target_data *data = dev->data;

	if (reg == REG_IPSR0) {
		return read_ipsr(dev);
	} else if (reg == REG_SOVR0) {
		return data->sovr;
	} else if (reg >= REG_CR_BASE && reg <= REG_CR_END) {
		return data->cr[reg - REG_CR_BASE];
	} else if (reg == REG_LBVIDR) {
		return IT8801_VENDOR_LO;
	} else if (reg == REG_HBVIDR) {
		return IT8801_VENDOR_HI;
	}
	/* Unimplemented registers read as 0 */
	return 0x00;
}

static void reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	struct it8801_target_data *data = dev->data;
	const struct it8801_target_config *cfg = dev->config;

	if (reg == REG_SOVR0) {
		data->sovr = val;
		apply_sovr(dev);
	} else if (reg >= REG_CR_BASE && reg <= REG_CR_END) {
		uint8_t pin = reg - REG_CR_BASE;
		data->cr[pin] = val;
		if (pin < cfg->ngpios) {
			gpio_flags_t flags;
			if (val & CR_GPIODIR) {
				flags = GPIO_OUTPUT;
				flags |= (data->sovr & BIT(pin))
					 ? GPIO_OUTPUT_HIGH
					 : GPIO_OUTPUT_LOW;
			} else {
				flags = GPIO_INPUT;
			}
			gpio_pin_configure_dt(&cfg->gpios[pin], flags);
		}
	}
	/* Writes to read-only or unimplemented registers are silently ignored */
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
		/* First byte is always the register address */
		data->reg_addr = val;
		data->reg_addr_valid = true;
	} else {
		/* Subsequent bytes are data */
		reg_write(data->dev, data->reg_addr, val);
		data->reg_addr++;
	}
	return 0;
}

static int cb_read_requested(struct i2c_target_config *config, uint8_t *val)
{
	struct it8801_target_data *data =
		CONTAINER_OF(config, struct it8801_target_data, i2c_cfg);

	if (!data->reg_addr_valid) {
		*val = 0x00;
		return 0;
	}

	*val = reg_read(data->dev, data->reg_addr);
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
	/* Nothing to do on STOP */
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

	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Configure all listed GPIOs as inputs initially */
	for (int i = 0; i < cfg->ngpios; i++) {
		if (!gpio_is_ready_dt(&cfg->gpios[i])) {
			LOG_ERR("GPIO %d not ready", i);
			return -ENODEV;
		}
		gpio_pin_configure_dt(&cfg->gpios[i], GPIO_INPUT);
	}

	data->i2c_cfg.address = cfg->bus.addr;
	data->i2c_cfg.callbacks = &target_callbacks;

	int ret = i2c_target_register(cfg->bus.bus, &data->i2c_cfg);
	if (ret) {
		LOG_ERR("Failed to register I2C target: %d", ret);
		return ret;
	}

	LOG_INF("IT8801 emulation at 0x%02x, %d GPIOs",
		cfg->bus.addr, cfg->ngpios);
	return 0;
}

/* ---------- device instantiation ----------------------------------- */

#define GPIO_DT_SPEC_BY_IDX(inst, idx) \
	GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst), gpios, idx)

#define IT8801_GPIO_SPEC(inst, idx) \
	COND_CODE_1(DT_INST_PROP_HAS_IDX(inst, gpios, idx), \
		(GPIO_DT_SPEC_BY_IDX(inst, idx),), ())

#define IT8801_TARGET_INIT(inst)                                               \
	static const struct gpio_dt_spec it8801_gpios_##inst[] = {             \
		IT8801_GPIO_SPEC(inst, 0)                                      \
		IT8801_GPIO_SPEC(inst, 1)                                      \
		IT8801_GPIO_SPEC(inst, 2)                                      \
		IT8801_GPIO_SPEC(inst, 3)                                      \
		IT8801_GPIO_SPEC(inst, 4)                                      \
		IT8801_GPIO_SPEC(inst, 5)                                      \
		IT8801_GPIO_SPEC(inst, 6)                                      \
		IT8801_GPIO_SPEC(inst, 7)                                      \
	};                                                                     \
                                                                               \
	static struct it8801_target_data it8801_data_##inst;                   \
                                                                               \
	static const struct it8801_target_config it8801_cfg_##inst = {         \
		.bus = I2C_DT_SPEC_INST_GET(inst),                             \
		.gpios = it8801_gpios_##inst,                                  \
		.ngpios = ARRAY_SIZE(it8801_gpios_##inst),                     \
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
