package(default_visibility = ["//visibility:public"])

load("//firmware/boards/shared:rules.bzl", "legacy_freertos_and_stm32_lib")
load("//firmware/boards/shared/legacy_usb:rules.bzl", "legacy_usb_lib")

cc_binary(
    name = "elf",
    # Note: It is normal and OK to put headers in `srcs`. Headers in `hdrs` represent
    #       the public interface of the library (hence why `cc_binary` has no `hdrs`
    #       field): https://docs.bazel.build/versions/master/be/c-cpp.html#cc_binary.srcs
    srcs = glob(
        [
            "*.c",
            "*.h",
        ],
    ),
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
        # We manually add include paths here, because if added in `includes`, it
        # appears that they are relative to the current directory, not absolute
        # paths from the project root (which is what we need).
        # This is in-line with the recommendations in the bazel documentation as of
        # 2020-03-18:
        # https://docs.bazel.build/versions/master/be/c-cpp.html#cc_binary.includes
        "-isystem firmware/boards/shared/legacy_freertos/include",
        "-isystem firmware/boards/shared/legacy_stm32lib/include",
        "-isystem firmware/boards/shared/legacy_usb/include",
    ],
    includes = [
        "cdcacm/include",
    ],
    linkopts = [
        "-T$(location :legacy_stm32f405.ld)",
        "-Wl,--build-id=none",
        "-Wl,--gc-sections",
        "-nostdlib",
    ],
    linkstatic = True,
    restricted_to = ["//cc_toolchain:stm32f4"],
    deps = [
        ":legacy_stm32f405.ld",
        ":usb",
    ],
)

genrule(
    name = "bin",
    srcs = [
        ":elf",
        "@arm_developer_gcc//:objcopy",
    ],
    outs = ["legacy_dongle_stm32f4.bin"],
    cmd = "\n".join([
        "$(location @arm_developer_gcc//:objcopy) -Obinary $(location :elf) $@",
    ]),
    restricted_to = ["//cc_toolchain:stm32f4"],
)

filegroup(
    name = "freertos_config_headers",
    srcs = [
        "FreeRTOSConfig.h",
    ],
)

legacy_freertos_and_stm32_lib(
    name = "freertos",
    freertos_config_headers = ":freertos_config_headers",
    includes = [
        ".",
    ],
)

legacy_usb_lib(
    name = "usb",
    freertos_lib = ":freertos",
)
