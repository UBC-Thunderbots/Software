package(default_visibility = ["//visibility:public"])

cc_library(
    name = "motorcontrol",
    srcs = [
        "motorcontrol.c",
    ],
    hdrs = [
        "motorcontrol.h",
    ],
    deps = [
        "//firmware:mc_config",
        "//firmware:mdv6_main_hooks",
        "//firmware/stm32f0xx:stm32f0xx",
    ],
)
