package(default_visibility = ["//visibility:public"])

cc_library(
    name = "validation_functions",
    srcs = [
        "friendly_scored_validation.cpp",
        "robot_received_ball_validation.cpp",
        "robots_slowed_down_validation.cpp",
    ],
    hdrs = [
        "friendly_scored_validation.h",
        "robot_received_ball_validation.h",
        "robots_slowed_down_validation.h",
    ],
    deps = [
        "//software/geom/algorithms",
        "//software/simulated_tests/validation:validation_function",
        "//software/world",
        "@boost//:coroutine2",
    ],
)
