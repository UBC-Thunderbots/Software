package(default_visibility = ["//visibility:public"])

cc_library(
    name = "constants",
    srcs = ["constants.c"],
    hdrs = ["constants.h"],
)

cc_library(
    name = "robot_constants",
    hdrs = ["robot_constants.h"],
    deps = [
        "//firmware/shared:tbots_math",
    ],
)
