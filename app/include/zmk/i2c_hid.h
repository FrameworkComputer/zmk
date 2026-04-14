/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

int zmk_i2c_hid_send_keyboard_report(void);
int zmk_i2c_hid_send_consumer_report(void);
#if IS_ENABLED(CONFIG_ZMK_POINTING)
int zmk_i2c_hid_send_mouse_report(void);
#endif // IS_ENABLED(CONFIG_ZMK_POINTING)
