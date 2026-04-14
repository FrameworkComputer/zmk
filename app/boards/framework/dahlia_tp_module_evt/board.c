/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Dahlia Touchpad Module board init.
 */

#include <zephyr/init.h>

static int dahlia_tp_module_board_init(void)
{
	return 0;
}

SYS_INIT(dahlia_tp_module_board_init, PRE_KERNEL_1, 0);
