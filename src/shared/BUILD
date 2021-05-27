package(default_visibility = ["//visibility:public"])

cc_library(
    name = "constants",
    srcs = [],
    hdrs = ["constants.h"],
    deps = [],
    alwayslink = True,
)

cc_library(
    name = "robot_constants",
    srcs = [
        "2015_robot_constants.c",
        "2015_wheel_constants.c",
    ],
    hdrs = [
        "2015_robot_constants.h",
        "2015_wheel_constants.h",
        "robot_constants.h",
    ],
    deps = [
        ":constants",
        "//firmware/shared/math:tbots_math",
    ],
)
