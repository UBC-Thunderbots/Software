package(default_visibility = ["//visibility:public"])

cc_library(
    name = "network",
    srcs = ["network.cpp"],
    hdrs = ["network.h"],
    deps = [
        ":packet_tracker",
        "//proto:tbots_cc_proto",
        "//shared:robot_constants",
        "//software/logger",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "@boost//:asio",
    ],
)

cc_library(
    name = "packet_tracker",
    srcs = ["packet_tracker.cpp"],
    hdrs = ["packet_tracker.h"],
    deps = [
        "//software/logger",
    ],
)
