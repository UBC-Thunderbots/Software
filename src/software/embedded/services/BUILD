package(default_visibility = ["//visibility:public"])

cc_library(
    name = "motor",
    srcs = ["motor.cpp"],
    hdrs = ["motor.h"],
    defines =
        select({
            "//software/embedded:build_trinamic": ["TRINAMIC_MOTOR_BOARD"],
            "//software/embedded:build_stspin": ["STSPIN_MOTOR_BOARD"],
        }),
    deps = [
        "//proto:tbots_cc_proto",
        "//shared:robot_constants",
        "//software/embedded/gpio",
        "//software/embedded/motor_controller",
        "//software/embedded/motor_controller:motor_board",
        "//software/embedded/motor_controller:motor_fault_indicator",
        "//software/logger",
        "//software/physics:euclidean_to_wheel",
        "//software/util/scoped_timespec_timer",
        "@eigen",
    ],
)

cc_library(
    name = "imu",
    srcs = ["imu.cpp"],
    hdrs = ["imu.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//software/geom:angular_velocity",
        "//software/logger",
        "@eigen",
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
