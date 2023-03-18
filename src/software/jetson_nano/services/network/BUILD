package(default_visibility = ["//visibility:public"])

cc_library(
    name = "network",
    srcs = ["network.cpp"],
    hdrs = ["network.h"],
    deps = [
        ":proto_tracker",
        "//proto:tbots_cc_proto",
        "//shared:robot_constants",
        "//software/logger",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "@boost//:asio",
    ],
)

cc_library(
    name = "proto_tracker",
    srcs = ["proto_tracker.cpp"],
    hdrs = ["proto_tracker.h"],
    deps = [
        "//software/logger",
    ],
)

cc_test(
    name = "proto_tracker_test",
    srcs = ["proto_tracker_test.cpp"],
    deps = [
        ":proto_tracker",
        "//shared/test_util:tbots_gtest_main",
    ],
)
