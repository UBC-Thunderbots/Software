package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proto_multicast_listener",
    hdrs = [
        "proto_multicast_listener.h",
        "proto_multicast_listener.tpp",
    ],
    deps = [
        "//software/logger",
        "@boost//:asio",
    ],
)

cc_library(
    name = "proto_multicast_sender",
    hdrs = [
        "proto_multicast_sender.h",
        "proto_multicast_sender.tpp",
    ],
    deps = [
        "//software/logger",
        "@boost//:asio",
    ],
)
