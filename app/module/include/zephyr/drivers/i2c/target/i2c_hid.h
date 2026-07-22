/**
 * @file
 *
 * @brief Public APIs for the I2C HID Target driver.
 */

/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_I2C_TARGET_I2C_HID_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2C_TARGET_I2C_HID_H_

#include <zephyr/device.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief I2C HID Target Driver API
 * @defgroup i2c_hid_target_api I2C HID Target Driver API
 * @ingroup io_interfaces
 * @{
 */

/** HID report types used in GET_REPORT / SET_REPORT commands */
enum i2c_hid_report_type {
	I2C_HID_REPORT_TYPE_INPUT = 1,
	I2C_HID_REPORT_TYPE_OUTPUT = 2,
	I2C_HID_REPORT_TYPE_FEATURE = 3,
};

/**
 * @brief Callback invoked when the host sends an output report.
 *
 * @param dev      Pointer to the I2C HID target device.
 * @param data     Pointer to the output report data (after the length prefix).
 * @param len      Length of the output report data in bytes.
 */
typedef void (*i2c_hid_output_report_cb_t)(const struct device *dev,
					   const uint8_t *data, uint16_t len);

/**
 * @brief Callback invoked when the host sends a SET_REPORT command.
 *
 * @param dev      Pointer to the I2C HID target device.
 * @param type     Report type (input, output, or feature).
 * @param id       Report ID.
 * @param data     Pointer to the report data.
 * @param len      Length of the report data in bytes.
 */
typedef void (*i2c_hid_set_report_cb_t)(const struct device *dev,
					enum i2c_hid_report_type type,
					uint8_t id, const uint8_t *data,
					uint16_t len);

/**
 * @brief Callback invoked when the host sends a GET_REPORT command.
 *
 * The application should fill @p buf with the requested report data
 * (without the 2-byte length prefix) and return the number of bytes written.
 *
 * @param dev      Pointer to the I2C HID target device.
 * @param type     Report type (input, output, or feature).
 * @param id       Report ID.
 * @param buf      Buffer to fill with report data.
 * @param max_len  Maximum number of bytes that can be written to @p buf.
 *
 * @retval >=0     Number of bytes written to @p buf.
 * @retval <0      Negative error code on failure.
 */
typedef int (*i2c_hid_get_report_cb_t)(const struct device *dev,
				       enum i2c_hid_report_type type,
				       uint8_t id, uint8_t *buf,
				       uint16_t max_len);

/**
 * Called after the host has read the reset marker (the empty report a HID
 * reset produces). The app should submit a fresh input report here: the
 * reset may have destroyed an unread report, and this resyncs the host.
 */
typedef void (*i2c_hid_reset_read_cb_t)(const struct device *dev);

/** Application callbacks for I2C HID target events */
struct i2c_hid_target_callbacks {
	i2c_hid_output_report_cb_t output_report;
	i2c_hid_set_report_cb_t set_report;
	i2c_hid_get_report_cb_t get_report;
	i2c_hid_reset_read_cb_t reset_read;
};

/**
 * @brief Set the HID report descriptor.
 *
 * Must be called before registering the device with
 * @ref i2c_target_driver_register. The length must match the
 * @c report-desc-length devicetree property.
 *
 * @param dev   Pointer to the I2C HID target device.
 * @param desc  Pointer to the report descriptor bytes. The data must remain
 *              valid for the lifetime of the device.
 * @param len   Length of the report descriptor in bytes.
 *
 * @retval 0       On success.
 * @retval -EINVAL If @p len does not match the DT-configured length.
 */
int i2c_hid_target_set_report_desc(const struct device *dev,
				   const uint8_t *desc, uint16_t len);

/**
 * @brief Register application callbacks.
 *
 * @param dev   Pointer to the I2C HID target device.
 * @param cb    Pointer to the callback structure. Must remain valid for the
 *              lifetime of the device. May be NULL to clear callbacks.
 *
 * @retval 0 On success.
 */
int i2c_hid_target_set_callbacks(const struct device *dev,
				 const struct i2c_hid_target_callbacks *cb);

/**
 * @brief Submit an input report to be read by the host.
 *
 * Builds a length-prefixed input report in a staging buffer, atomically
 * swaps it into the active position, and asserts the IRQ GPIO to notify
 * the host. May be called from thread context.
 *
 * @param dev        Pointer to the I2C HID target device.
 * @param report_id  HID report ID (0 if only one input report).
 * @param data       Pointer to the report data (without report ID or length
 *                   prefix).
 * @param len        Length of @p data in bytes.
 *
 * @retval 0       On success.
 * @retval -EINVAL If the total report exceeds @c max-input-length.
 * @retval -ENODEV If the report descriptor has not been set.
 */
int i2c_hid_target_submit_report(const struct device *dev, uint8_t report_id,
				 const uint8_t *data, uint16_t len);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2C_TARGET_I2C_HID_H_ */
