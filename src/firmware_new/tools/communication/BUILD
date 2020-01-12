package(default_visibility = ["//visibility:public"])

cc_library(
    name = "robot_communicator",
    srcs = ["robot_communicator.cpp"],
    hdrs = ["robot_communicator.h"],
    deps = [
        "//firmware_new/tools/communication/transfer_media:transfer_medium",
        "//firmware_new/tools/communication/transfer_media:network_medium",
        "//software/multithreading:thread_safe_buffer",
        "//firmware_new/proto:fw_cc_proto",
    ],
)
