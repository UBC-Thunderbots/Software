package(default_visibility = ["//visibility:public"])

cc_library(
    name = "network",
    srcs = ["network.cpp"],
    hdrs = ["network.h"],
    deps = [
        ":primitive_tracker",
        "//proto:tbots_cc_proto",
        "//shared:robot_constants",
        "//software/logger:network_logger",
        "//software/networking/udp:threaded_proto_udp_listener",
        "//software/networking/udp:threaded_proto_udp_sender",
        "//software/time:duration",
        "//software/time:timestamp",
        "//software/world:robot_state",
        "@boost//:asio",
        "@tracy",
    ],
)

cc_library(
    name = "primitive_tracker",
    srcs = ["primitive_tracker.cpp"],
    hdrs = ["primitive_tracker.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//shared:constants",
        "//software/logger",
    ],
)

cc_test(
    name = "primitive_tracker_test",
    srcs = ["primitive_tracker_test.cpp"],
    deps = [
        ":primitive_tracker",
        "//shared/test_util:tbots_gtest_main",
    ],
)
