package(default_visibility = ["//visibility:public"])

# TODO: remove all `strip_include_prefix` once firmware has been setup to have includes
#       from top-level

cc_library(
    name = "chicker",
    srcs = ["chicker.c"],
    hdrs = ["chicker.h"],
    strip_include_prefix = "/firmware/main",
)

cc_library(
    name = "dribbler",
    srcs = ["dribbler.c"],
    hdrs = ["dribbler.h"],
    strip_include_prefix = "/firmware/main",
)

cc_library(
    name = "firmware_ball",
    srcs = ["firmware_ball.c"],
    hdrs = ["firmware_ball.h"],
    strip_include_prefix = "/firmware/main",
)

cc_library(
    name = "firmware_robot",
    srcs = ["firmware_robot.c"],
    hdrs = ["firmware_robot.h"],
    strip_include_prefix = "/firmware/main",
    deps = [
        ":chicker",
        ":dribbler",
        ":wheel",
    ],
)

cc_library(
    name = "firmware_world",
    srcs = ["firmware_world.c"],
    hdrs = ["firmware_world.h"],
    strip_include_prefix = "/firmware/main",
    deps = [
        ":firmware_ball",
        ":firmware_robot",
    ],
)

cc_library(
    name = "wheel",
    srcs = ["wheel.c"],
    hdrs = ["wheel.h"],
    strip_include_prefix = "/firmware/main",
)
