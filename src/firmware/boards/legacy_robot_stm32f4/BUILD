package(default_visibility = ["//visibility:public"])

load("//firmware/boards/shared:rules.bzl", "legacy_shared_libs")

cc_binary(
    name = "elf",
    # Note: It is normal and OK to put headers in `srcs`. Headers in `hdrs` represent
    #       the public interface of the library (hence why `cc_binary` has no `hdrs`
    #       field): https://docs.bazel.build/versions/master/be/c-cpp.html#cc_binary.srcs
    srcs = glob(
        [
            "*.c",
            "*.h",
            "legacy_cdcacm/*.c",
            "legacy_cdcacm/include/legacy_cdcacm.h",
            "io/*.c",
            "io/*.h",
            "math/*.c",
            "math/*.h",
            "upgrade/*.c",
            "upgrade/*.h",
            "util/*.c",
            "util/*.h",
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
        "-isystem firmware/boards/shared/legacy_cdcacm/include",
        "-isystem firmware/boards/shared/legacy_usb/include",
        # We don't want to mark warnings as errors for legacy code
        "-Wno-error",
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
        "//firmware/app/control",
        "//firmware/app/logger",
        "//firmware/app/primitives:primitive_manager",
        "//firmware/app/world:firmware_world",
        "//firmware/shared:physics",
        "//firmware/shared:util",
        "//shared:constants",
        "//shared:robot_constants",
        "//shared/proto:tbots_nanopb_proto",
        "@nanopb",
    ],
)

genrule(
    name = "bin",
    srcs = [
        ":elf",
        "@arm_developer_gcc//:objcopy",
    ],
    outs = ["legacy_robot_stm32f4.bin"],
    cmd = "\n".join([
        "$(location @arm_developer_gcc//:objcopy) -Obinary $(location :elf) $@",
    ]),
    restricted_to = ["//cc_toolchain:stm32f4"],
)

filegroup(
    name = "freertos_config_headers",
    srcs = [
        "FreeRTOSConfig.h",
        "priority.h",
    ],
)

legacy_shared_libs(
    name = "legacy_shared_libs",
    freertos_config_headers = ":freertos_config_headers",
    includes = [
        ".",
    ],
)
