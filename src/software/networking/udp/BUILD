package(default_visibility = ["//visibility:public"])

cc_library(
    name = "network_utils",
    srcs = [
        "network_utils.cpp",
    ],
    hdrs = [
        "network_utils.h",
    ],
)

cc_test(
    name = "network_utils_test",
    srcs = [
        "network_utils_test.cpp",
    ],
    deps = [
        ":network_utils",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "proto_udp_listener",
    hdrs = [
        "proto_udp_listener.hpp",
    ],
    visibility = ["//visibility:private"],
    deps = [
        ":network_utils",
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
        ":network_utils",
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

cc_test(
    name = "threaded_proto_udp_listener_test",
    srcs = [
        "threaded_proto_udp_listener_test.cpp",
    ],
    deps = [
        ":threaded_proto_udp_listener",
        "//shared/test_util:tbots_gtest_main",
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

cc_test(
    name = "threaded_proto_udp_sender_test",
    srcs = [
        "threaded_proto_udp_sender_test.cpp",
    ],
    deps = [
        ":threaded_proto_udp_sender",
        "//shared/test_util:tbots_gtest_main",
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
