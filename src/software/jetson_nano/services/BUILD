package(default_visibility = ["//visibility:public"])

cc_library(
    name = "service",
    hdrs = ["service.h"],
)

cc_library(
    name = "motor",
    srcs = ["motor.cpp"],
    hdrs = ["motor.h"],
    deps = [
        ":service",
        "//proto:tbots_cc_proto",
        "//shared:robot_constants",
        "//software/logger",
        "@trinamic",
    ],
)

cc_library(
    name = "network",
    srcs = ["network.cpp"],
    hdrs = ["network.h"],
    deps = [
        ":service",
        "//proto:tbots_cc_proto",
        "//software/logger",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "//shared:robot_constants",
        "@boost//:asio",
    ],
)

cc_test(
    name = "receive_test",
    srcs = [
        "network_service_test.cpp",
    ],
    deps = [
        ":network",
        "//software/backend:simulator_backend",
        "//shared/test_util:tbots_gtest_main",
    ],
)