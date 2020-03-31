# Copyright (c) 2020 Ioannis Konstantelias
#
# SPDX-License-Identifier: Apache-2.0
#
cmake_minimum_required(VERSION 3.13.1)

include($ENV{PRJ_DIR}/zephyr/cmake/boilerplate.cmake)

# Set shield
set(SHIELD x_nucleo_iks01a3)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(compare_baros)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
