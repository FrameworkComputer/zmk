/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/target/i2c_hid.h>

#include <zmk/hid.h>
#include <zmk/endpoints_types.h>

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#include <zmk/hid_indicators.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static const struct device *i2c_hid_dev = DEVICE_DT_GET(DT_NODELABEL(i2c_hid));

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
/* Handle output reports from host (e.g. Caps Lock LED state) */
static void i2c_hid_output_report_cb(const struct device *dev,
                                     const uint8_t *data, uint16_t len)
{
    /* Expected format: [report_id, led_bits] */
    if (len < 2 || data[0] != ZMK_HID_REPORT_ID_LEDS) {
        return;
    }

    struct zmk_hid_led_report_body report = { .leds = data[1] };
    struct zmk_endpoint_instance endpoint = { .transport = ZMK_TRANSPORT_I2C_HID };
    zmk_hid_indicators_process_report(&report, endpoint);
}

static const struct i2c_hid_target_callbacks i2c_hid_callbacks = {
    .output_report = i2c_hid_output_report_cb,
};
#endif

int zmk_i2c_hid_send_keyboard_report(void) {
    struct zmk_hid_keyboard_report *report = zmk_hid_get_keyboard_report();

    int ret = i2c_hid_target_submit_report(i2c_hid_dev, ZMK_HID_REPORT_ID_KEYBOARD,
                                           (const uint8_t *)&report->body,
                                           sizeof(report->body));
    if (ret) {
        printk("[I2C TX KB] ERROR: %d\n", ret);
    }
    return ret;
}

int zmk_i2c_hid_send_consumer_report(void) {
    struct zmk_hid_consumer_report *report = zmk_hid_get_consumer_report();

    int ret = i2c_hid_target_submit_report(i2c_hid_dev, ZMK_HID_REPORT_ID_CONSUMER,
                                        (const uint8_t *)&report->body,
                                        sizeof(report->body));
    if (ret) {
        printk("[I2C TX Consumer] ERROR: %d\n", ret);
    }
    return ret;
}

#if IS_ENABLED(CONFIG_ZMK_POINTING)
int zmk_i2c_hid_send_mouse_report(void) {
    struct zmk_hid_mouse_report *report = zmk_hid_get_mouse_report();

    return i2c_hid_target_submit_report(i2c_hid_dev, ZMK_HID_REPORT_ID_MOUSE,
                                        (const uint8_t *)&report->body,
                                        sizeof(report->body));
}
#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

static int zmk_i2c_hid_init(void) {
    if (!device_is_ready(i2c_hid_dev)) {
        printk("[I2C HID] device not ready\n");
        return -ENODEV;
    }

    printk("[I2C HID] report desc size: %u (DT expects %u)\n",
           (unsigned int)sizeof(zmk_hid_report_desc),
           DT_PROP(DT_NODELABEL(i2c_hid), report_desc_length));

    int ret = i2c_hid_target_set_report_desc(i2c_hid_dev,
                                             zmk_hid_report_desc,
                                             sizeof(zmk_hid_report_desc));
    if (ret < 0) {
        printk("[I2C HID] Failed to set report desc: %d (update DT to %u)\n",
               ret, (unsigned int)sizeof(zmk_hid_report_desc));
        return ret;
    }

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
    i2c_hid_target_set_callbacks(i2c_hid_dev, &i2c_hid_callbacks);
#endif

    ret = i2c_target_driver_register(i2c_hid_dev);
    if (ret < 0) {
        printk("[I2C HID] Failed to register target: %d\n", ret);
        return ret;
    }

    printk("[I2C HID] target registered at 0x%02x\n",
           DT_REG_ADDR(DT_NODELABEL(i2c_hid)));

    /* Dump I2C1 registers for debug */
    volatile uint32_t *i2c1 = (volatile uint32_t *)0x40005400;
    printk("[I2C1] CR1=0x%08x CR2=0x%08x OAR1=0x%08x OAR2=0x%08x\n",
           i2c1[0], i2c1[1], i2c1[2], i2c1[3]);
    printk("[I2C1] TIMINGR=0x%08x ISR=0x%08x\n",
           i2c1[4], i2c1[6]);

    /* Also check RCC APB1ENR1 bit 21 = I2C1 clock enable */
    volatile uint32_t *rcc_apb1enr1 = (volatile uint32_t *)0x40021058;
    printk("[RCC] APB1ENR1=0x%08x (I2C1EN=%d)\n",
           *rcc_apb1enr1, (*rcc_apb1enr1 >> 21) & 1);

    /* Check GPIOB MODER for PB6/PB7 — should be 0x2 (AF mode) */
    volatile uint32_t *gpiob_moder = (volatile uint32_t *)0x50000400;
    volatile uint32_t *gpiob_afrl = (volatile uint32_t *)0x50000420;
    printk("[GPIOB] MODER=0x%08x (PB6=%d PB7=%d) AFRL=0x%08x\n",
           *gpiob_moder,
           (*gpiob_moder >> 12) & 3,
           (*gpiob_moder >> 14) & 3,
           *gpiob_afrl);

    return 0;
}

SYS_INIT(zmk_i2c_hid_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
