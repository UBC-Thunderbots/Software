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
        "//shared:constants",
        "//shared/proto:fw_cc_proto",
        "//software/backend/output/wifi/communication:robot_communicator",
        "//software/backend/output/wifi/communication/transfer_media:network_medium",
        "//software/backend/output/wifi:wifi_output",
        "@boost//:asio",
        "@com_google_protobuf//:protobuf",
    ],
)
