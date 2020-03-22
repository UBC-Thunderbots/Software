package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gpio_pin",
    srcs = ["gpio_pin.c"],
    hdrs = ["gpio_pin.h"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        "//firmware_new/boards/frankie_v1:hal",
    ],
)

cc_library(
    name = "pwm_pin",
    srcs = ["pwm_pin.c"],
    hdrs = ["pwm_pin.h"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        "//firmware_new/boards/frankie_v1:hal",
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
