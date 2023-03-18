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
        "//software/util/scoped_timespec_timer",
        "//software/physics:euclidean_to_three_wheel",
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
