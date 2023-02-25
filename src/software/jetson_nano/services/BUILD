package(default_visibility = ["//visibility:public"])

cc_library(
    name = "motor",
    srcs = ["motor.cpp"],
    hdrs = ["motor.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//shared:robot_constants",
        "//software/jetson_nano:gpio",
        "//software/logger",
        "//software/physics:euclidean_to_wheel",
        "//software/util/scoped_timespec_timer",
        "@eigen",
        "@trinamic",
    ],
)

cc_library(
    name = "power",
    srcs = ["power.cpp"],
    hdrs = ["power.h"],
    linkopts = ["-latomic"],  # Necessary due to <atomic> not containing load/store for all sizes
    deps = [
        "//shared/uart_framing",
        "//software/logger",
        "//software/uart:boost_uart_communication",
        "@boost//:asio",
        "@boost//:filesystem",
    ],
)

cc_library(
    name = "network",
    srcs = ["network.cpp"],
    hdrs = ["network.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//shared:robot_constants",
        "//software/logger",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "@boost//:asio",
    ],
)

cc_test(
    name = "robot_auto_test",
    srcs = ["robot_auto_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
    ],
)
