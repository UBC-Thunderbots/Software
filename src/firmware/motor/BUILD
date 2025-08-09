package(default_visibility = ["//visibility:public"])

load("@bazel_embedded//tools/openocd:defs.bzl", "openocd_flash")

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

openocd_flash(
    name = "mdv6_firmware_flash",
    device_configs = [
        # Part of the STM32F0 family
        # https://www.st.com/resource/en/reference_manual/rm0091-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
        "target/stm32f0x.cfg",
    ],
    # .stripped strips debug symbols to reduce the size of the binary.
    # see: https://bazel.build/reference/be/c-cpp#cc_binary
    image = ":mdv6_firmware_main.stripped",
    interface_configs = [
        "interface/stlink.cfg",  # The ST-Link V2 programmer is used to flash the firmware.
    ],
)
