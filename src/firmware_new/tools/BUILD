sh_binary(
    name = "debug_firmware_on_arm_board",
    srcs = ["debug_firmware_on_arm_board.sh"],
    data = [
        "//firmware_new/boards/frankie_v1:frankie_v1_main",
        "@arm_developer_gcc//:gdb",
    ],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)

# bazel pin mapper
filegroup(
    name = "firmware_c_files",
    srcs = glob(["firmware_new/boards/frankie_v1/*c",]),
)

filegroup(
    name = "firmware_h_files",
    srcs = glob(["firmware_new/boards/frankie_v1/*h",]),
)

sh_binary(
    name = "regen",
    srcs = ["regen.sh"],
    data = ["//firmware_new/boards/frankie_v1:frankie_v1.ioc"],
    args = [":c_files", ":h_files"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)
