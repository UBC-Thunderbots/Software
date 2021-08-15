package(default_visibility = ["//visibility:public"])

load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

cc_library(
    name = "gpio_pin",
    srcs = ["gpio_pin.c"],
    hdrs = ["gpio_pin.h"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        "//firmware/boards/robot_stm32h7:hal",
    ],
)

cc_library(
    name = "pwm_pin",
    srcs = ["pwm_pin.c"],
    hdrs = ["pwm_pin.h"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        "//firmware/boards/robot_stm32h7:hal",
    ],
)

cc_library(
    name = "allegro_a3931_motor_driver",
    srcs = ["allegro_a3931_motor_driver.c"],
    hdrs = ["allegro_a3931_motor_driver.h"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        ":gpio_pin",
        ":pwm_pin",
    ],
)

cc_library(
    name = "infineon_TLE009_E1000_angle_sensor",
    srcs = ["infineon_TLE5009_E1000_angle_sensor.c"],
    hdrs = ["infineon_TLE5009_E1000_angle_sensor.h"],
)

cc_test(
    name = "infineon_TLE009_E1000_angle_sensor_test",
    srcs = ["infineon_TLE5009_E1000_angle_sensor_test.cpp"],
    deps = [
        "//firmware/boards/robot_stm32h7/io:infineon_TLE009_E1000_angle_sensor",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "drivetrain",
    srcs = ["drivetrain.c"],
    hdrs = ["drivetrain.h"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [":drivetrain_unit"],
)

cc_library(
    name = "drivetrain_unit",
    srcs = ["drivetrain_unit.c"],
    hdrs = ["drivetrain_unit.h"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [":allegro_a3931_motor_driver"],
)

cc_library(
    name = "proto_multicast_communication",
    srcs = [
        "proto_multicast_communication_profile.c",
        "proto_multicast_communication_tasks.c",
    ],
    hdrs = [
        "proto_multicast_communication_profile.h",
        "proto_multicast_communication_tasks.h",
    ],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        ":ublox_odinw262_communicator",
        "//firmware/app/logger",
        "//firmware/boards/robot_stm32h7:hal",
        "//firmware/boards/robot_stm32h7:lwip",
        "//shared/proto:tbots_nanopb_proto",
        "@nanopb",
    ],
)

cc_library(
    name = "network_logger",
    srcs = ["network_logger.c"],
    hdrs = ["network_logger.h"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        "//firmware/boards/robot_stm32h7:hal",
        "//firmware/boards/robot_stm32h7:lwip",
        "//firmware/boards/robot_stm32h7:rtos",
        "//firmware/boards/robot_stm32h7/io:proto_multicast_communication",
        "//shared/proto:tbots_nanopb_proto",
        "@nanopb",
    ],
    alwayslink = True,
)

cc_library(
    name = "uart_logger",
    srcs = ["uart_logger.c"],
    hdrs = ["uart_logger.h"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        "//firmware/boards/robot_stm32h7:hal",
        "//firmware/boards/robot_stm32h7:rtos",
        "//firmware/boards/robot_stm32h7:usart",
        "//shared/proto:tbots_nanopb_proto",
        "@nanopb",
    ],
    alwayslink = True,
)

cc_library(
    name = "ublox_odinw262_communicator",
    srcs = ["ublox_odinw262_communicator.c"],
    hdrs = ["ublox_odinw262_communicator.h"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        ":gpio_pin",
        "//firmware/app/logger",
        "//firmware/boards/robot_stm32h7:hal",
        "//firmware/boards/robot_stm32h7:rtos",
        "//firmware/boards/robot_stm32h7:usart",
        "//firmware/boards/robot_stm32h7/io:ublox_odinw262_communicator_buffer_utils",
        "//shared:constants",
        "//shared/proto:tbots_nanopb_proto",
        "@nanopb",
    ],
    alwayslink = True,
)

cc_library(
    name = "ublox_odinw262_communicator_buffer_utils",
    srcs = ["ublox_odinw262_communicator_buffer_utils.c"],
    hdrs = ["ublox_odinw262_communicator_buffer_utils.h"],
    alwayslink = True,
)

# external cmake repo for property based testing
cmake(
    name = "rapidcheck",
    lib_source = "@rapidcheck//:all",
    out_static_libs = ["librapidcheck.a"],
)

cc_test(
    name = "ublox_odinw262_communicator_buffer_utils_test",
    srcs = ["ublox_odinw262_communicator_buffer_utils_test.cpp"],
    deps = [
        ":rapidcheck",
        "//firmware/boards/robot_stm32h7/io:ublox_odinw262_communicator_buffer_utils",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "power_monitor",
    srcs = ["power_monitor.c"],
    hdrs = ["power_monitor.h"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [":INA226_power_monitor_driver"],
)

cc_library(
    name = "INA226_power_monitor_driver",
    srcs = ["INA226_power_monitor_driver.c"],
    hdrs = ["INA226_power_monitor_driver.h"],
    copts = [
        # This library uses hal and includes ST headers, so we can't mark warnings as errors
        # This was adapted from: https://github.com/macgeorge/STM32-example-codes
        "-Wno-error",
    ],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        "//firmware/boards/robot_stm32h7:cmsis",
        "//firmware/boards/robot_stm32h7:hal",
    ],
)
