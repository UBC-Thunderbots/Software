package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gpio",
    srcs = [
        "gpio_char_dev.cpp",
        "gpio_sysfs.cpp",
    ],
    hdrs = [
        "gpio.h",
        "gpio_char_dev.h",
        "gpio_sysfs.h",
    ],
    deps = [
        "//software/logger",
        "//software/logger:network_logger",
        "//software/util/make_enum",
    ],
)
