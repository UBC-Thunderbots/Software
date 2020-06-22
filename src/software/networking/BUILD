package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proto_multicast_listener",
    hdrs = [
        "proto_multicast_listener.h",
        "proto_multicast_listener.tpp",
    ],
    visibility = ["//visibility:private"],
    deps = [
        "//software/logger",
        "@boost//:asio",
    ],
)

cc_library(
    name = "threaded_proto_multicast_listener",
    hdrs = [
        "threaded_proto_multicast_listener.h",
        "threaded_proto_multicast_listener.tpp",
    ],
    deps = [
        ":proto_multicast_listener",
        "@boost//:asio",
    ],
)

cc_library(
    name = "proto_multicast_sender",
    hdrs = [
        "proto_multicast_sender.h",
        "proto_multicast_sender.tpp",
    ],
    visibility = ["//visibility:private"],
    deps = [
        "//software/logger",
        "@boost//:asio",
    ],
)

cc_library(
    name = "threaded_proto_multicast_sender",
    hdrs = [
        "threaded_proto_multicast_sender.h",
        "threaded_proto_multicast_sender.tpp",
    ],
    deps = [
        ":proto_multicast_sender",
        "@boost//:asio",
    ],
)
