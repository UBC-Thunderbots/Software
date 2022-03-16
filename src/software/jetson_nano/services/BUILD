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
        "//software/jetson_nano:gpio",
        "//software/logger",
        "@trinamic",
    ],
)

cc_library(
    name = "power",
    srcs = ["power.cpp"],
    hdrs = ["power.h"],
    linkopts = ["-latomic"],  # Necessary due to <atomic> not containing load/store for all sizes
    deps = [
        ":service",
        "//shared/uart_framing",
        "//software/logger",
        "//software/uart:boost_uart_communication",
        "@boost//:asio",
    ],
)

cc_library(
    name = "network",
    srcs = ["network.cpp"],
    hdrs = ["network.h"],
    deps = [
        ":service",
        "//proto:tbots_cc_proto",
        "//shared:robot_constants",
        "//software/logger",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "@boost//:asio",
    ],
)
