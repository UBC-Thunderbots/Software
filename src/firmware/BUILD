package(default_visibility = ["//visibility:public"])

load("//firmware_new:rules.bzl", "stm32h7_hal_library_files_genrule")

cc_binary(
    name = "firmware_old_main",
    srcs = ["main/main.c"],
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
        "freertos",
        "main",
    ],
    linkopts = [
        "-T$(location //firmware:stm32f405.ld)",
        "-Wl,--build-id=none",
        "-Wl,--gc-sections",
        "-nostdlib",
    ],
    linkstatic = True,
    restricted_to = ["//cc_toolchain:stm32f4"],
    deps = [
        ":firmware_old_main_lib",
        ":stm32f405.ld",
    ],
)

genrule(
    name = "firmware_old_main_bin",
    srcs = [
        ":firmware_old_main",
        "@arm_developer_gcc//:objcopy",
    ],
    outs = ["firmware_old_main.bin"],
    cmd = "\n".join([
        "$(location @arm_developer_gcc//:objcopy) -Obinary $(location :firmware_old_main) $@",
    ]),
    restricted_to = ["//cc_toolchain:stm32f4"],
)

cc_library(
    name = "firmware_old_main_lib",
    srcs = glob(
        [
            "usb/**/*.c",
            "main/**/*.c",
            "cdcacm/**/*.c",
            "stm32lib/**/*.c",
            "freertos/source/**/*.c",
        ],
        exclude = [
            "main/main.c",
        ],
    ),
    hdrs = glob(
        [
            "usb/**/*.h",
            "main/**/*.h",
            "stm32lib/**/*.h",
            "freertos/include/*.h",
            "cdcacm/include/*.h",
        ],
    ),
    defines = [
        "STM32LIB_USE_FREERTOS",
    ],
    includes = [
        "cdcacm/include",
        "freertos/include",
        "freertos/vendor/portable/GCC/ARM_CM4F",
        "main",
        "stm32lib/include",
        "usb/include",
    ],
    restricted_to = ["//cc_toolchain:stm32f4"],
    deps = [
        "//shared:constants",
        "//shared:robot_constants",
    ],
    # This is _very_ important. If we don't do this, bazel will not link in the
    # interrupt handling code, which *will not* cause an error, and *will* cause
    # interrupts to stop working
    alwayslink = True,
)
