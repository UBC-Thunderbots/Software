sh_binary(
    name = "debug_firmware_on_arm_board",
    srcs = ["debug_firmware_on_arm_board.sh"],
    data = [
        "//firmware_new/boards/frankie_v1:frankie_v1_main",
        "@arm_developer_gcc//:gdb",
    ],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)
