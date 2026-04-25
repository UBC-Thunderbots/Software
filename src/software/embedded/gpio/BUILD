package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gpio",
    srcs = [
        "gpio.cpp",
        "gpio_char_dev.cpp",
    ],
    hdrs = [
        "gpio.h",
        "gpio_char_dev.h",
    ],
    deps = [
        "//software/logger",
        "//software/logger:network_logger",
        "//software/util/make_enum",
    ],
)
