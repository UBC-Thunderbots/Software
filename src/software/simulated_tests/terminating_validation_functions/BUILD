package(default_visibility = ["//visibility:public"])

cc_library(
    name = "terminating_validation_functions",
    testonly = True,
    srcs = [
        "friendly_scored_validation.cpp",
        "robot_at_position_validation.cpp",
        "robot_received_ball_validation.cpp",
    ],
    hdrs = [
        "friendly_scored_validation.h",
        "robot_at_position_validation.h",
        "robot_received_ball_validation.h",
    ],
    deps = [
        "//software/geom/algorithms",
        "//software/simulated_tests/validation:validation_function",
        "//software/world",
        "@boost//:coroutine2",
    ],
)
