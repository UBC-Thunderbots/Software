package(default_visibility = ["//visibility:public"])

cc_library(
    name = "motor",
    srcs = ["motor.cpp"],
    hdrs = ["motor.h"],
    defines =
        select({
            "//software/embedded:build_nano": ["NANO"],
            "//software/embedded:build_pi": ["PI"],
            "//software/embedded:build_limited": ["LIMITED"],
        }),
    deps = [
        "//proto:tbots_cc_proto",
        "//shared:robot_constants",
        "//software/embedded:gpio",
        "//software/embedded/constants",
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

cc_binary(
    name = "robot_auto_test",
    srcs = ["robot_auto_test.cpp"],
    linkopts = [
        "-lpthread",
        "-lrt",
    ],
    deps = [
        ":motor",
        ":power",
        "//proto/message_translation:tbots_geometry",
        "//proto/primitive:primitive_msg_factory",
        "//shared:robot_constants",
        "//software/embedded:primitive_executor",
        "//software/logger",
        "@trinamic",
    ],
)
