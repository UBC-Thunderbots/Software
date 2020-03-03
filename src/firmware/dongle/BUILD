Package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "firmware_old_radio_dongle_main",
    srcs = ["main.c"],
    copts = [
        "-std=gnu99",
        "-O2",
        "-mfloat-abi=hard",
        "-mlittle-endian",
        "-mcpu=cortex-m4",
        "-mfpu=fpv4-sp-d16",
        "-mthumb",
        "-ggdb3",
        "-fno-common",
        "-ffunction-sections",
        "-static",
        "-Wall",
        "-Wextra",
        "-Wdouble-promotion",
        "-Wpointer-arith",
    ],
    includes = [
        "main",
    ],
    linkopts = [
        "-T$(location //firmware:stm32f405.ld)",
        "-Wl,--build-id=none",
        "-Wl,--gc-sections",
    ],
    linkstatic = True,
    restricted_to = ["//cc_toolchain:stm32f4"],
    deps = [
        ":firmware_old_radio_dongle_lib",
        "//firmware:stm32f405.ld",
    ],
)

genrule(
    name = "firmware_old_radio_dongle_bin",
    srcs = [
        ":firmware_old_radio_dongle_main",
        "@arm_developer_gcc//:objcopy",
    ],
    outs = ["firmware_old_radio_dongble.bin"],
    cmd = "\n".join([
        "$(location @arm_developer_gcc//:objcopy) -Obinary $(location :firmware_old_radio_dongle_main) $@",
    ]),
    restricted_to = ["//cc_toolchain:stm32f4"],
)

cc_library(
    name = "firmware_old_radio_dongle_lib",
    srcs = glob(
        [
            "*.c",
        ],
        exclude = [
            "main.c",
        ],
    ),
    hdrs = glob(
        [
            "*.h",
        ],
    ),
    includes = [
        "dongle",
    ],
    restricted_to = ["//cc_toolchain:stm32f4"],
    deps = [
        "//firmware:dongle_freertos",
        "//firmware:firmware_old_radio_dongle_usb_lib",
    ],
    # This is _very_ important. If we don't do this, bazel will not link in the
    # interrupt handling code, which *will not* cause an error, and *will* cause
    # interrupts to stop working
    alwayslink = True,
)

cc_library(
    name = "firmware_old_radio_dongle_freertos_config",
    hdrs = ["FreeRTOSConfig.h"],
)
