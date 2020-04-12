package(default_visibility = ["//visibility:public"])

cc_library(
    name = "robot_communicator",
    srcs = ["robot_communicator.cpp"],
    hdrs = ["robot_communicator.h"],
    restricted_to = ["//cc_toolchain:k8"],
    deps = [
        "//firmware_new/proto:fw_cc_proto",
        "//firmware_new/tools/communication/transfer_media:network_medium",
        "//firmware_new/tools/communication/transfer_media:transfer_medium",
        "//software/logger",
        "//software/multithreading:thread_safe_buffer",
        "@boost//:asio",
    ],
)
