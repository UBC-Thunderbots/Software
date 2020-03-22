package(default_visibility = ["//visibility:public"])

cc_library(
    name = "robot_communicator",
    srcs = ["robot_communicator.cpp"],
    hdrs = ["robot_communicator.h"],
    deps = [
        "//shared/proto:fw_cc_proto",
        "//software/backend/output/wifi/communication/transfer_media:network_medium",
        "//software/backend/output/wifi/communication/transfer_media:transfer_medium",
        "//software/multithreading:thread_safe_buffer",
        "@boost//:asio",
    ],
)

cc_library(
    name = "protobuf_primitive_visitor",
    srcs = ["protobuf_primitive_visitor.cpp"],
    hdrs = ["protobuf_primitive_visitor.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//shared:constants",
        "//shared/proto:fw_cc_proto",
        "//software/ai/primitive:all_primitives",
        "//software/ai/primitive:primitive_visitor",
    ],
)
