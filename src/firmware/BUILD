package(default_visibility = ["//visibility:public"])

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
        ],
        exclude = [
            "main/main.c",
        ],
    ),
    hdrs = glob(
        [
            "usb/**/*.h",
            "main/**/*.h",
            "cdcacm/include/*.h",
        ],
    ),
    includes = [
        ".",
        "cdcacm/include",
        "main",
        "usb/include",
    ],
    restricted_to = ["//cc_toolchain:stm32f4"],
    deps = [
        ":main_freertos",
        "//firmware/main/app/control",
        "//firmware/main/app/primitives:primitive_manager",
        "//firmware/main/app/world:firmware_world",
        "//firmware/main/shared:physics",
        "//firmware/main/shared:util",
        "//shared:constants",
        "//shared:robot_constants",
    ],
    # This is _very_ important. If we don't do this, bazel will not link in the
    # interrupt handling code, which *will not* cause an error, and *will* cause
    # interrupts to stop working
    alwayslink = True,
)

cc_library(
    name = "main_freertos",
    srcs = glob([
        "freertos/source/**/*.c",
        "stm32lib/**/*.c",
    ]),
    hdrs = glob([
        "freertos/include/*.h",
        "stm32lib/include/**/*.h",
    ]) + [
        "main/FreeRTOSConfig.h",
        "main/priority.h",
    ],
    copts = [
        # We don't want to mark warnings as errors for 3rd party code
        "-Wno-error",
    ],
    defines = [
        "STM32LIB_USE_FREERTOS",
    ],
    includes = [
        "freertos/include",
        "main",
        "stm32lib/include",
        "stm32lib/include/registers",
    ],
    restricted_to = ["//cc_toolchain:stm32f4"],
)
