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
    name = "motor",
    srcs = ["motor.c"],
    hdrs = ["motor.h"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        ":gpio_pin",
        ":pwm_pin",
    ],
)
