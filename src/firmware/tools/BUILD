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
    name = "stm32h7_firmware",
    srcs = ["stm32h7_firmware.sh"],
    data = [
        "//firmware/boards/robot_stm32h7:robot_stm32h7_main",
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
