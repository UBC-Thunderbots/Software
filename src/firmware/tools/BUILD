package(default_visibility = ["//visibility:private"])

sh_binary(
    name = "legacy_flash_firmware",
    srcs = ["legacy_flash_firmware.sh"],
    data = [
        "//firmware/boards/legacy_dongle_stm32f4:bin",
        "//firmware/boards/legacy_robot_stm32f4:bin",
    ],
    restricted_to = ["//cc_toolchain:stm32f4"],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)

sh_binary(
    name = "debug_firmware_on_stm32h7",
    srcs = ["debug_firmware_on_stm32h7.sh"],
    data = [
        "//firmware_new/boards/frankie_v1:frankie_v1_main",
        "@arm_developer_gcc//:gdb",
    ],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)

sh_binary(
    name = "cubemx_regen",
    srcs = ["cubemx_regen.sh"],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)
