package(default_visibility = ["//visibility:public"])

cc_library(
    name = "io",
    srcs = [
        "gpio_pin.c",
        "motor.c",
    ],
    hdrs = [
        "gpio_pin.h",
        "motor.h",
    ],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        "//firmware_new/boards/frankie_v1:frankie_v1_main_lib",
    ],
)
