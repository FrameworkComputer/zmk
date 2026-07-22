# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_include_directories(${APPLICATION_SOURCE_DIR}/include)
zephyr_library_sources(board.c)

# Regenerate dahlia_git_version.h (git hash + dirty flag) on every build.
add_custom_target(dahlia_git_version
    COMMAND ${CMAKE_COMMAND} -DREPO=${CMAKE_CURRENT_SOURCE_DIR}
            -DOUT=${CMAKE_CURRENT_BINARY_DIR}/dahlia_git_version.h
            -P ${CMAKE_CURRENT_SOURCE_DIR}/git_version.cmake)
zephyr_library_include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_dependencies(${ZEPHYR_CURRENT_LIBRARY} dahlia_git_version)
