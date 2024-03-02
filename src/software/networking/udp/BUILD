package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proto_udp_client",
    hdrs = [
        "proto_udp_client.hpp",
    ],
    deps = [
        "//software/logger",
        "//software/util/typename",
    ],
)

cc_library(
    name = "proto_udp_listener",
    hdrs = [
        "proto_udp_listener.hpp",
    ],
    visibility = ["//visibility:private"],
    deps = [
        "//software/logger",
        "//software/util/typename",
    ],
)

cc_library(
    name = "udp_sender",
    srcs = [
        "udp_sender.cpp",
    ],
    hdrs = [
        "udp_sender.h",
    ],
    visibility = ["//visibility:private"],
    deps = [
        "@boost//:asio",
    ],
)

cc_library(
    name = "threaded_proto_udp_listener",
    hdrs = [
        "threaded_proto_udp_listener.hpp",
    ],
    deps = [
        ":proto_udp_listener",
        "@boost//:asio",
    ],
)

cc_library(
    name = "threaded_proto_udp_sender",
    hdrs = [
        "threaded_proto_udp_sender.hpp",
    ],
    deps = [
        ":threaded_udp_sender",
    ],
)

cc_library(
    name = "threaded_udp_sender",
    srcs = [
        "threaded_udp_sender.cpp",
    ],
    hdrs = [
        "threaded_udp_sender.h",
    ],
    deps = [
        ":udp_sender",
    ],
)
