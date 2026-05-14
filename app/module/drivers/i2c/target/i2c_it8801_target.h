/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public API for the IT8801 register emulation target.
 */

#ifndef ZEPHYR_DRIVERS_I2C_TARGET_IT8801_H_
#define ZEPHYR_DRIVERS_I2C_TARGET_IT8801_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

/**
 * Callback invoked for each SOVR bit that changed value due to a host
 * write. Fired one call per bit transition (no call for unchanged bits).
 *
 * @param dev    The IT8801 target device.
 * @param port   SOVR port (0..2).
 * @param pin    Bit position within the port (0..7).
 * @param level  New bit value (true = 1, false = 0).
 */
typedef void (*it8801_target_change_cb_t)(const struct device *dev,
					  uint8_t port, uint8_t pin,
					  bool level);

/**
 * Register (or clear, with NULL) the SOVR change callback. Only one
 * callback is supported.
 */
void it8801_target_set_callback(const struct device *dev,
				it8801_target_change_cb_t cb);

/**
 * Return the current SOVR shadow value for a port. Useful for reading
 * the initial state after init.
 */
uint8_t it8801_target_get_sovr(const struct device *dev, uint8_t port);

#endif /* ZEPHYR_DRIVERS_I2C_TARGET_IT8801_H_ */
