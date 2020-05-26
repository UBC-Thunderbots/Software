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

sh_binary(
    name = "cubemx_regen",
    srcs = ["cubemx_regen.sh"],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)

cc_binary(
    name = "send_proto_over_udp",
    srcs = ["send_proto_over_udp.cpp"],
    restricted_to = ["//cc_toolchain:k8"],
    deps = [
        "//firmware_new/boards/frankie_v1:constants",
        "//firmware_new/proto:fw_cc_proto",
        "//software/logger",
        "//software/networking:proto_multicast_listener",
        "//software/networking:proto_multicast_sender",
        "@boost//:asio",
        "@com_google_protobuf//:protobuf",
    ],
)
