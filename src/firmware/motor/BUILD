package(default_visibility = ["//visibility:public"])

filegroup(
    name = "mdv6_firmware_srcs",
    srcs = glob(["**/*.c"])
)

filegroup(
    name = "mdv6_firmware_hdrs",
    srcs = glob(["**/*.h"])
)

cc_binary(
    name = "mdv6_firmware_main",
    deps = [
        ":mdv6_firmware",
    ],
    target_compatible_with = [
        "@platforms//cpu:armv6-m",
        "@platforms//os:none",
    ],
)

cc_library(
    name = "mdv6_firmware",
    srcs = glob(["**/*.c"]),
    hdrs = glob(["**/*.h"]),
    defines = [
        # Our MCU is the STM32F0251: https://www.st.com/resource/en/datasheet/stspin32f0251.pdf
        "STM32F031x6",
        # Use the Low-Layer APIs because it gives us more control over the hardware (as opposed to the HAL).
        # https://www.st.com/content/ccc/resource/technical/document/user_manual/56/32/53/cb/69/86/49/0e/DM00223149.pdf/files/DM00223149.pdf/jcr:content/translations/en.DM00223149.pdf
        "USE_FULL_LL_DRIVER",
    ],
)

cc_library(
    name = "mc_config",
    srcs = ["mc_config.c"],
    hdrs = ["mc_config.h"],
    deps = [
        ":mdv6_main_hooks",
        "//firmware/mcsdk:pid_regulator",
    ],
)

cc_library(
    name = "mdv6_main_hooks",
    hdrs = ["main.h"],
    deps = [
        "//firmware/stm32f0xx:stm32f0xx",
        "//firmware/motorcontrol:motorcontrol",
    ],
)
