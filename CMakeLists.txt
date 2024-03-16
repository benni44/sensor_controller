# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ti_cc13x2_cc26x2_sensor_controller)

include_directories(inc)
file(GLOB app_sources src/*.c)
file(GLOB scif_sources inc/scif/*.c)
target_sources(app PRIVATE ${app_sources} ${scif_sources})