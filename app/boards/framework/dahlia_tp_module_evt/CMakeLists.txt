# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_include_directories(${APPLICATION_SOURCE_DIR}/include)
zephyr_library_sources(../dahlia_tp_module/board.c)
