package(default_visibility = ["//visibility:public"])

cc_library(
    name = "position_controller",
    srcs = [
        "position_controller.cpp",
    ],
    hdrs = [
        "position_controller.h",
    ],
    deps = [
        ":pid_controller",
        "//software/ai/navigator/trajectory:trajectory_path",
    ]
)

cc_library(
    name = "orientation_controller",
    srcs = [
        "orientation_controller.cpp",
    ],
    hdrs = [
        "orientation_controller.h",
    ],
    deps = [
        ":pid_controller",
        "//software/ai/navigator/trajectory:bang_bang_trajectory_1d_angular",
        "//software/geom:angle",
        "//software/geom:angular_velocity",
    ]
)

cc_library(
    name = "pid_controller",
    srcs = [
        "pid_controller.cpp",
    ],
    hdrs = [
        "pid_controller.h",
    ],
)

cc_test(
    name = "pid_controller_test",
    srcs = [
        "pid_controller_test.cpp",
    ],
    deps = [
        ":pid_controller",
        "//shared/test_util:tbots_gtest_main",
    ],
)
