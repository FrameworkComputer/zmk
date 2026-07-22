/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C HID Target driver — device/target side of Microsoft HID over I2C
 * protocol (v1.00).
 */

#define DT_DRV_COMPAT zephyr_i2c_target_hid

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <errno.h>

#include <zephyr/drivers/i2c/target/i2c_hid.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_hid_target);

/* HID over I2C descriptor length (30 bytes) */
#define HID_DESC_LEN 30

/* Register addresses (reported inside the HID descriptor) */
#define REG_REPORT_DESC 0x0002
#define REG_INPUT       0x0003
#define REG_OUTPUT      0x0004
#define REG_COMMAND     0x0005
#define REG_DATA        0x0006

/* Command opcodes */
#define CMD_RESET       0x01
#define CMD_GET_REPORT  0x02
#define CMD_SET_REPORT  0x03
#define CMD_SET_POWER   0x08

/* Internal state for the I2C target callback state machine */
enum i2c_hid_state {
	STATE_IDLE,
	STATE_REG_KNOWN,
	STATE_WRITE_COMMAND,
	STATE_WRITE_OUTPUT,
	STATE_WRITE_DATA,
};

/* Which register the host addressed */
enum i2c_hid_reg {
	REG_ID_NONE,
	REG_ID_HID_DESC,
	REG_ID_REPORT_DESC,
	REG_ID_INPUT,
	REG_ID_OUTPUT,
	REG_ID_COMMAND,
	REG_ID_DATA,
};

/* 30-byte HID-over-I2C descriptor, all little-endian */
struct i2c_hid_desc {
	uint16_t wHIDDescLength;
	uint16_t bcdVersion;
	uint16_t wReportDescLength;
	uint16_t wReportDescRegister;
	uint16_t wInputRegister;
	uint16_t wMaxInputLength;
	uint16_t wOutputRegister;
	uint16_t wMaxOutputLength;
	uint16_t wCommandRegister;
	uint16_t wDataRegister;
	uint16_t wVendorID;
	uint16_t wProductID;
	uint16_t wVersionID;
	uint32_t reserved;
} __packed;

struct i2c_hid_target_config {
	struct i2c_dt_spec bus;
	struct gpio_dt_spec irq_gpio;
	uint16_t hid_desc_register;
	uint16_t vid;
	uint16_t pid;
	uint16_t version_id;
	uint16_t max_input_length;
	uint16_t max_output_length;
	uint16_t report_desc_length;
};

struct i2c_hid_target_data {
	/* Zephyr I2C target config — must be first for CONTAINER_OF */
	struct i2c_target_config config;

	const struct device *dev;

	/* Pre-built 30-byte HID descriptor */
	struct i2c_hid_desc hid_desc;

	/* Report descriptor (pointer set by application) */
	const uint8_t *report_desc;

	/* Application callbacks */
	const struct i2c_hid_target_callbacks *app_cb;

	/* Double-buffered input reports */
	uint8_t *input_buf_a;
	uint8_t *input_buf_b;
	/* Atomic pointer: which buffer the ISR reads from */
	atomic_ptr_t active_input_buf;
	/* Flag: an input report is pending for the host */
	atomic_t input_report_pending;

	/* GET_REPORT response buffer (data register) */
	uint8_t *data_buf;
	uint16_t data_buf_len;

	/* Write buffer — collects bytes from host for command/output */
	uint8_t *write_buf;
	uint16_t write_buf_len;
	uint16_t write_idx;

	/* State machine (ISR context only) */
	enum i2c_hid_state state;
	enum i2c_hid_reg addressed_reg;

	/* Register address accumulator (2-byte LE) */
	uint8_t reg_addr_bytes[2];
	uint8_t reg_addr_idx;

	/* Read pointer + length for ISR read path */
	const uint8_t *read_buf;
	uint16_t read_len;
	uint16_t read_idx;

	/* Work items — all GPIO and command processing deferred to thread */
	struct k_work irq_assert_work;
	struct k_work irq_deassert_work;
	struct k_work cmd_work;
	struct k_work reset_work;
	struct k_work output_work;
	struct k_work reset_read_work;

	/* Buffer holding the reset marker; non-NULL until the host reads it */
	const uint8_t *sentinel_buf;

	/* Snapshot of command bytes for deferred processing */
	uint8_t cmd_buf[4];
	uint8_t cmd_len;

	/* Snapshot of output report for deferred processing */
	uint16_t output_snapshot_len;

	/* Pending SET_REPORT context: set by cmd_handler when it sees a
	 * SET_REPORT opcode, consumed by output_handler when the report data
	 * arrives in the next transaction (via the DATA or OUTPUT register). */
	bool pending_set_report;
	uint8_t pending_set_type;
	uint8_t pending_set_id;
};

/* ---------- helpers --------------------------------------------------- */

static enum i2c_hid_reg resolve_register(const struct i2c_hid_target_config *cfg,
					 uint16_t reg_addr)
{
	if (reg_addr == cfg->hid_desc_register) {
		return REG_ID_HID_DESC;
	}
	switch (reg_addr) {
	case REG_REPORT_DESC: return REG_ID_REPORT_DESC;
	case REG_INPUT:       return REG_ID_INPUT;
	case REG_OUTPUT:      return REG_ID_OUTPUT;
	case REG_COMMAND:     return REG_ID_COMMAND;
	case REG_DATA:        return REG_ID_DATA;
	default:              return REG_ID_NONE;
	}
}

/* ---------- work handlers (thread context) ----------------------------- */

static void irq_assert_handler(struct k_work *work)
{
	struct i2c_hid_target_data *data =
		CONTAINER_OF(work, struct i2c_hid_target_data, irq_assert_work);
	const struct i2c_hid_target_config *cfg = data->dev->config;

	gpio_pin_set_dt(&cfg->irq_gpio, 1);
}

static void irq_deassert_handler(struct k_work *work)
{
	struct i2c_hid_target_data *data =
		CONTAINER_OF(work, struct i2c_hid_target_data, irq_deassert_work);
	const struct i2c_hid_target_config *cfg = data->dev->config;

	gpio_pin_set_dt(&cfg->irq_gpio, 0);

	/*
	 * Race protection: if a new report came in between the STOP
	 * handler clearing input_report_pending and us running, the
	 * pending flag is 1 again. The new submit queued an assert_work
	 * but it was a no-op (IRQ already asserted), so without this
	 * re-assert the EC ends up with IRQ deasserted while a report
	 * sits in the buffer — no edge to wake the EC.
	 *
	 * Hold HIGH for 20 µs before re-asserting so the EC's input
	 * deglitch filter registers it as a clean deassert and the
	 * subsequent LOW as a real falling edge. With just two back-to-
	 * back GPIO writes (~125 ns each at 16 MHz) the EC's filter may
	 * eat the pulse entirely and see continuous LOW.
	 */
	if (atomic_get(&data->input_report_pending)) {
		k_busy_wait(20);
		gpio_pin_set_dt(&cfg->irq_gpio, 1);
	}
}

static void reset_handler(struct k_work *work)
{
	struct i2c_hid_target_data *data =
		CONTAINER_OF(work, struct i2c_hid_target_data, reset_work);

	/*
	 * Prepare a reset-complete sentinel: a 2-byte input report with
	 * length = 0x0000. Build in the staging buffer (the one NOT
	 * currently active), then swap.
	 */
	uint8_t *staging = (atomic_ptr_get(&data->active_input_buf) ==
			    data->input_buf_a)
			   ? data->input_buf_b
			   : data->input_buf_a;

	staging[0] = 0x00;
	staging[1] = 0x00;
	atomic_ptr_set(&data->active_input_buf, staging);
	atomic_set(&data->input_report_pending, 1);
	data->sentinel_buf = staging;

	/* Assert IRQ to tell host the reset is complete */
	k_work_submit(&data->irq_assert_work);
}

/* The host has read the reset marker — let the app resync its state. */
static void reset_read_handler(struct k_work *work)
{
	struct i2c_hid_target_data *data =
		CONTAINER_OF(work, struct i2c_hid_target_data, reset_read_work);

	if (data->app_cb && data->app_cb->reset_read) {
		data->app_cb->reset_read(data->dev);
	}
}

static void cmd_handler(struct k_work *work)
{
	struct i2c_hid_target_data *data =
		CONTAINER_OF(work, struct i2c_hid_target_data, cmd_work);
	const struct i2c_hid_target_config *cfg = data->dev->config;

	uint8_t byte0 = data->cmd_buf[0];
	uint8_t opcode = data->cmd_buf[1];
	uint8_t report_type = (byte0 >> 4) & 0x0F;
	uint8_t report_id = byte0 & 0x0F;

	/* Extended report ID */
	if (report_id == 0x0F && data->cmd_len >= 3) {
		report_id = data->cmd_buf[2];
	}

	switch (opcode) {
	case CMD_RESET:
		LOG_DBG("HID RESET command");
		k_work_submit(&data->reset_work);
		break;

	case CMD_GET_REPORT: {
		LOG_DBG("GET_REPORT type=%u id=%u", report_type, report_id);
		int ret = 0;

		if (data->app_cb && data->app_cb->get_report) {
			/* data_buf layout: [len_lo, len_hi, report_id, ...data] */
			uint16_t max_payload = cfg->max_input_length - 2;

			if (report_id != 0) {
				max_payload -= 1; /* room for report_id byte */
			}

			uint8_t *payload_start = data->data_buf + 2;

			if (report_id != 0) {
				payload_start[0] = report_id;
				payload_start++;
				max_payload = MIN(max_payload,
						  (uint16_t)(cfg->max_input_length - 3));
			}

			ret = data->app_cb->get_report(
				data->dev,
				(enum i2c_hid_report_type)report_type,
				report_id, payload_start, max_payload);
		}

		if (ret < 0) {
			/* Error: respond with zero-length */
			data->data_buf[0] = 0x00;
			data->data_buf[1] = 0x00;
			data->data_buf_len = 2;
		} else {
			uint16_t total = 2 + (report_id != 0 ? 1 : 0) + ret;

			data->data_buf[0] = total & 0xFF;
			data->data_buf[1] = (total >> 8) & 0xFF;
			data->data_buf_len = total;
		}
		break;
	}

	case CMD_SET_REPORT:
		/*
		 * SET_REPORT: the host follows this command with a write to the
		 * DATA (or OUTPUT) register containing the report payload.
		 * Stash the type/id so output_handler knows this next payload is
		 * a SET_REPORT (and to which report) rather than an unsolicited
		 * Output register write.
		 */
		LOG_DBG("SET_REPORT type=%u id=%u (data follows)", report_type, report_id);
		data->pending_set_report = true;
		data->pending_set_type = report_type;
		data->pending_set_id = report_id;
		break;

	case CMD_SET_POWER:
		LOG_DBG("SET_POWER state=%u", report_id);
		break;

	default:
		LOG_WRN("Unknown HID command opcode=0x%02x", opcode);
		break;
	}
}

static void output_handler(struct k_work *work)
{
	struct i2c_hid_target_data *data =
		CONTAINER_OF(work, struct i2c_hid_target_data, output_work);

	/*
	 * write_buf contains the raw register data.
	 * First 2 bytes are the length prefix, followed by the report data.
	 */
	uint16_t len = data->output_snapshot_len;

	if (len <= 2) {
		data->pending_set_report = false;
		return;
	}

	if (data->pending_set_report) {
		/* This payload followed a SET_REPORT command — dispatch as
		 * a typed set_report if the app registered a handler. */
		if (data->app_cb && data->app_cb->set_report) {
			data->app_cb->set_report(
				data->dev,
				(enum i2c_hid_report_type)data->pending_set_type,
				data->pending_set_id,
				data->write_buf + 2, len - 2);
		}
		data->pending_set_report = false;
	} else if (data->app_cb && data->app_cb->output_report) {
		/* Unsolicited Output register write (legacy Output reports
		 * such as keyboard LED state). */
		data->app_cb->output_report(data->dev, data->write_buf + 2, len - 2);
	}
}

/* ---------- I2C target callbacks (ISR context) ------------------------- */

static int hid_target_write_requested(struct i2c_target_config *config)
{
	struct i2c_hid_target_data *data =
		CONTAINER_OF(config, struct i2c_hid_target_data, config);

	data->state = STATE_IDLE;
	data->reg_addr_idx = 0;
	data->write_idx = 0;

	return 0;
}

static int hid_target_read_requested(struct i2c_target_config *config,
				     uint8_t *val)
{
	struct i2c_hid_target_data *data =
		CONTAINER_OF(config, struct i2c_hid_target_data, config);

	switch (data->addressed_reg) {
	case REG_ID_HID_DESC:
		data->read_buf = (const uint8_t *)&data->hid_desc;
		data->read_len = HID_DESC_LEN;
		break;

	case REG_ID_REPORT_DESC:
		data->read_buf = data->report_desc;
		data->read_len = data->report_desc
			? ((const struct i2c_hid_target_config *)data->dev->config)
				->report_desc_length
			: 0;
		break;

	case REG_ID_DATA:
		data->read_buf = data->data_buf;
		data->read_len = data->data_buf_len;
		break;

	case REG_ID_INPUT:
		/* Fall through — same as IDLE (host reads input report) */
	case REG_ID_NONE:
	default:
		/* Default read: serve the active input report buffer */
		data->read_buf = (const uint8_t *)atomic_ptr_get(
			&data->active_input_buf);
		data->read_len = ((const struct i2c_hid_target_config *)
				  data->dev->config)->max_input_length;
		break;
	}

	data->read_idx = 0;

	if (data->read_buf && data->read_len > 0) {
		*val = data->read_buf[0];
		data->read_idx = 1;
	} else {
		*val = 0x00;
	}

	return 0;
}

static int hid_target_read_processed(struct i2c_target_config *config,
				     uint8_t *val)
{
	struct i2c_hid_target_data *data =
		CONTAINER_OF(config, struct i2c_hid_target_data, config);

	if (data->read_buf && data->read_idx < data->read_len) {
		*val = data->read_buf[data->read_idx++];
	} else {
		*val = 0x00;
	}

	return 0;
}

static int hid_target_write_received(struct i2c_target_config *config,
				     uint8_t val)
{
	struct i2c_hid_target_data *data =
		CONTAINER_OF(config, struct i2c_hid_target_data, config);

	/* First 2 bytes: register address (little-endian) */
	if (data->reg_addr_idx < 2) {
		data->reg_addr_bytes[data->reg_addr_idx++] = val;

		if (data->reg_addr_idx == 2) {
			uint16_t reg_addr = (uint16_t)data->reg_addr_bytes[0] |
					    ((uint16_t)data->reg_addr_bytes[1] << 8);

			data->addressed_reg = resolve_register(
				data->dev->config, reg_addr);

			switch (data->addressed_reg) {
			case REG_ID_COMMAND:
				data->state = STATE_WRITE_COMMAND;
				data->write_idx = 0;
				break;
			case REG_ID_OUTPUT:
				data->state = STATE_WRITE_OUTPUT;
				data->write_idx = 0;
				break;
			case REG_ID_DATA:
				/* SET_REPORT payload lands here in the two-transaction
				 * (Windows-style) SET_REPORT flow: host wrote command
				 * first, now writes report data to the data register. */
				data->state = STATE_WRITE_DATA;
				data->write_idx = 0;
				break;
			default:
				data->state = STATE_REG_KNOWN;
				break;
			}
		}
		return 0;
	}

	/* Subsequent bytes: store into write_buf */
	switch (data->state) {
	case STATE_WRITE_COMMAND:
		if (data->write_idx < sizeof(data->cmd_buf)) {
			data->cmd_buf[data->write_idx++] = val;
		}
		break;

	case STATE_WRITE_OUTPUT:
	case STATE_WRITE_DATA:
		if (data->write_idx < data->write_buf_len) {
			data->write_buf[data->write_idx++] = val;
		}
		break;

	default:
		break;
	}

	return 0;
}

static int hid_target_stop(struct i2c_target_config *config)
{
	struct i2c_hid_target_data *data =
		CONTAINER_OF(config, struct i2c_hid_target_data, config);

	switch (data->state) {
	case STATE_WRITE_COMMAND:
		data->cmd_len = data->write_idx;
		k_work_submit(&data->cmd_work);
		break;

	case STATE_WRITE_OUTPUT:
	case STATE_WRITE_DATA:
		data->output_snapshot_len = data->write_idx;
		k_work_submit(&data->output_work);
		break;

	default:
		break;
	}

	/*
	 * If the host just read the input register and a report was pending,
	 * deassert IRQ.
	 *
	 * Race: a new submit can happen during the read transaction. The
	 * submit calls atomic_ptr_set on active_input_buf and re-arms
	 * pending=1. The atomic_cas below clears that, losing track of the
	 * new report. Detect by comparing the cached read_buf to the
	 * current active_input_buf — if they differ, a swap happened
	 * during the read, so there's still a fresh report and we must
	 * keep pending raised so the deassert handler re-asserts.
	 */
	if (data->addressed_reg == REG_ID_INPUT ||
	    data->addressed_reg == REG_ID_NONE) {
		bool swap_during_read =
			(atomic_ptr_get(&data->active_input_buf) !=
			 (atomic_ptr_val_t)data->read_buf);

		if (atomic_cas(&data->input_report_pending, 1, 0)) {
			if (swap_during_read) {
				atomic_set(&data->input_report_pending, 1);
			}
			k_work_submit(&data->irq_deassert_work);
		}

		/* Host consumed the reset marker: have the app resync. */
		if (data->read_idx > 0 &&
		    data->read_buf == data->sentinel_buf) {
			data->sentinel_buf = NULL;
			k_work_submit(&data->reset_read_work);
		}
	}

	/* Reset state for next transaction */
	data->state = STATE_IDLE;
	data->addressed_reg = REG_ID_NONE;
	data->reg_addr_idx = 0;

	return 0;
}

static const struct i2c_target_callbacks hid_target_callbacks = {
	.write_requested = hid_target_write_requested,
	.read_requested = hid_target_read_requested,
	.write_received = hid_target_write_received,
	.read_processed = hid_target_read_processed,
	.stop = hid_target_stop,
};

/* ---------- public API ------------------------------------------------ */

int i2c_hid_target_set_report_desc(const struct device *dev,
				   const uint8_t *desc, uint16_t len)
{
	const struct i2c_hid_target_config *cfg = dev->config;
	struct i2c_hid_target_data *data = dev->data;

	if (len != cfg->report_desc_length) {
		return -EINVAL;
	}

	data->report_desc = desc;

	return 0;
}

int i2c_hid_target_set_callbacks(const struct device *dev,
				 const struct i2c_hid_target_callbacks *cb)
{
	struct i2c_hid_target_data *data = dev->data;

	data->app_cb = cb;

	return 0;
}

int i2c_hid_target_submit_report(const struct device *dev, uint8_t report_id,
				 const uint8_t *report_data, uint16_t len)
{
	const struct i2c_hid_target_config *cfg = dev->config;
	struct i2c_hid_target_data *data = dev->data;

	if (!data->report_desc) {
		return -ENODEV;
	}

	/* Total wire length: 2 (len prefix) + optional report_id + data */
	uint16_t total = 2 + (report_id != 0 ? 1 : 0) + len;

	if (total > cfg->max_input_length) {
		return -EINVAL;
	}

	/*
	 * Build the report in the staging buffer (the one NOT currently
	 * pointed to by active_input_buf).
	 */
	uint8_t *staging = (atomic_ptr_get(&data->active_input_buf) ==
			    data->input_buf_a)
			   ? data->input_buf_b
			   : data->input_buf_a;

	staging[0] = total & 0xFF;
	staging[1] = (total >> 8) & 0xFF;

	uint16_t offset = 2;

	if (report_id != 0) {
		staging[offset++] = report_id;
	}

	memcpy(staging + offset, report_data, len);

	/* Atomically swap so the ISR sees a complete buffer */
	atomic_ptr_set(&data->active_input_buf, staging);
	atomic_set(&data->input_report_pending, 1);

	/* Defer GPIO assertion to thread context */
	k_work_submit(&data->irq_assert_work);

	return 0;
}

/* ---------- driver register / unregister ------------------------------ */

static int i2c_hid_target_register(const struct device *dev)
{
	const struct i2c_hid_target_config *cfg = dev->config;
	struct i2c_hid_target_data *data = dev->data;

	return i2c_target_register(cfg->bus.bus, &data->config);
}

static int i2c_hid_target_unregister(const struct device *dev)
{
	const struct i2c_hid_target_config *cfg = dev->config;
	struct i2c_hid_target_data *data = dev->data;

	return i2c_target_unregister(cfg->bus.bus, &data->config);
}

static const struct i2c_target_driver_api i2c_hid_api = {
	.driver_register = i2c_hid_target_register,
	.driver_unregister = i2c_hid_target_unregister,
};

/* ---------- init ------------------------------------------------------ */

static int i2c_hid_target_init(const struct device *dev)
{
	const struct i2c_hid_target_config *cfg = dev->config;
	struct i2c_hid_target_data *data = dev->data;
	int ret;

	data->dev = dev;

	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C controller device not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
		LOG_ERR("IRQ GPIO device not ready");
		return -ENODEV;
	}

	/* Configure IRQ GPIO: output, initially deasserted (inactive) */
	ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure IRQ GPIO: %d", ret);
		return ret;
	}

	/* Build the 30-byte HID descriptor from DT config */
	data->hid_desc.wHIDDescLength = HID_DESC_LEN;
	data->hid_desc.bcdVersion = 0x0100;
	data->hid_desc.wReportDescLength = cfg->report_desc_length;
	data->hid_desc.wReportDescRegister = REG_REPORT_DESC;
	data->hid_desc.wInputRegister = REG_INPUT;
	data->hid_desc.wMaxInputLength = cfg->max_input_length;
	data->hid_desc.wOutputRegister = REG_OUTPUT;
	data->hid_desc.wMaxOutputLength = cfg->max_output_length;
	data->hid_desc.wCommandRegister = REG_COMMAND;
	data->hid_desc.wDataRegister = REG_DATA;
	data->hid_desc.wVendorID = cfg->vid;
	data->hid_desc.wProductID = cfg->pid;
	data->hid_desc.wVersionID = cfg->version_id;
	data->hid_desc.reserved = 0;

	/* Initialize I2C target config */
	data->config.address = cfg->bus.addr;
	data->config.callbacks = &hid_target_callbacks;

	/* Initialize double-buffered input reports to zero-length */
	memset(data->input_buf_a, 0, cfg->max_input_length);
	memset(data->input_buf_b, 0, cfg->max_input_length);
	atomic_ptr_set(&data->active_input_buf, data->input_buf_a);
	atomic_set(&data->input_report_pending, 0);

	/* Initialize data_buf (GET_REPORT response) */
	data->data_buf_len = 0;

	/* Initialize work items */
	k_work_init(&data->irq_assert_work, irq_assert_handler);
	k_work_init(&data->irq_deassert_work, irq_deassert_handler);
	k_work_init(&data->cmd_work, cmd_handler);
	k_work_init(&data->reset_work, reset_handler);
	k_work_init(&data->output_work, output_handler);
	k_work_init(&data->reset_read_work, reset_read_handler);

	/* Initial state */
	data->state = STATE_IDLE;
	data->addressed_reg = REG_ID_NONE;
	data->reg_addr_idx = 0;

	return 0;
}

/* ---------- device instantiation -------------------------------------- */

#define I2C_HID_TARGET_INIT(inst)                                              \
	static uint8_t i2c_hid_input_a_##inst[DT_INST_PROP(inst,              \
							    max_input_length)];\
	static uint8_t i2c_hid_input_b_##inst[DT_INST_PROP(inst,              \
							    max_input_length)];\
	static uint8_t i2c_hid_data_buf_##inst[DT_INST_PROP(inst,             \
							     max_input_length)];\
	static uint8_t i2c_hid_write_buf_##inst[                               \
		MAX(DT_INST_PROP(inst, max_output_length),                     \
		    DT_INST_PROP(inst, max_input_length))];                    \
                                                                               \
	static struct i2c_hid_target_data i2c_hid_data_##inst = {              \
		.input_buf_a = i2c_hid_input_a_##inst,                         \
		.input_buf_b = i2c_hid_input_b_##inst,                         \
		.data_buf = i2c_hid_data_buf_##inst,                           \
		.write_buf = i2c_hid_write_buf_##inst,                         \
		.write_buf_len = sizeof(i2c_hid_write_buf_##inst),             \
	};                                                                     \
                                                                               \
	static const struct i2c_hid_target_config i2c_hid_cfg_##inst = {       \
		.bus = I2C_DT_SPEC_INST_GET(inst),                             \
		.irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),           \
		.hid_desc_register = DT_INST_PROP(inst, hid_desc_register),    \
		.vid = DT_INST_PROP(inst, vid),                                \
		.pid = DT_INST_PROP(inst, pid),                                \
		.version_id = DT_INST_PROP(inst, version_id),                  \
		.max_input_length = DT_INST_PROP(inst, max_input_length),      \
		.max_output_length = DT_INST_PROP(inst, max_output_length),    \
		.report_desc_length = DT_INST_PROP(inst, report_desc_length),  \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(inst,                                            \
			      &i2c_hid_target_init,                            \
			      NULL,                                            \
			      &i2c_hid_data_##inst,                            \
			      &i2c_hid_cfg_##inst,                             \
			      POST_KERNEL,                                     \
			      CONFIG_I2C_TARGET_INIT_PRIORITY,                 \
			      &i2c_hid_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_HID_TARGET_INIT)
