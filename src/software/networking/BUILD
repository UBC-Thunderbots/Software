package(default_visibility = ["//visibility:public"])

cc_library(
    name = "multicast_listener",
    srcs = ["multicast_listener.cpp"],
    hdrs = ["multicast_listener.h"],
    visibility = ["//visibility:private"],
    deps = [
        "//software/logger",
        "@boost//:asio",
    ],
)

cc_library(
    name = "nanopb_primitive_set_listener",
    srcs = ["nanopb_primitive_set_multicast_listener.cpp"],
    hdrs = ["nanopb_primitive_set_multicast_listener.h"],
    deps = [
        ":multicast_listener",
        "//shared/proto:tbots_nanopb_proto",
        "//software/logger",
        "//software/world:robot_state",
        "@nanopb",
    ],
)

cc_library(
    name = "proto_multicast_listener",
    hdrs = [
        "proto_multicast_listener.h",
        "proto_multicast_listener.tpp",
    ],
    visibility = ["//visibility:private"],
    deps = [
        ":multicast_listener",
        "//software/logger",
        "//software/util/typename",
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
    name = "threaded_nanopb_primitive_set_multicast_listener",
    hdrs = ["threaded_nanopb_primitive_set_multicast_listener.h"],
    deps = [
        ":nanopb_primitive_set_listener",
        ":threaded_network_listener",
        "@nanopb",
    ],
)

cc_library(
    name = "threaded_network_listener",
    hdrs = [
        "threaded_network_listener.h",
        "threaded_network_listener.tpp",
    ],
    deps = [
        "@boost//:asio",
    ],
)

cc_library(
    name = "threaded_proto_multicast_listener",
    hdrs = [
        "threaded_proto_multicast_listener.h",
    ],
    deps = [
        ":proto_multicast_listener",
        ":threaded_network_listener",
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
