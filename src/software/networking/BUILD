package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proto_udp_client",
    hdrs = [
        "proto_udp_client.h",
        "proto_udp_client.tpp",
    ],
    visibility = ["//visibility:private"],
    deps = [
        "//software/logger",
        "//software/util/typename",
    ],
)

cc_library(
    name = "threaded_proto_udp_listener",
    hdrs = [
        "threaded_proto_udp_listener.h",
        "threaded_proto_udp_listener.tpp",
    ],
    deps = [
        ":proto_udp_client",
        "@boost//:asio",
    ],
)

cc_library(
    name = "threaded_proto_udp_sender",
    hdrs = [
        "threaded_proto_udp_sender.h",
        "threaded_proto_udp_sender.tpp",
    ],
    deps = [
        ":proto_udp_client",
        "@boost//:asio",
    ],
)
