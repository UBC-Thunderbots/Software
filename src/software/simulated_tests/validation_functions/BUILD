package(default_visibility = ["//visibility:public"])

cc_library(
    name = "validation_functions",
    srcs = [
        "ball_in_rectangle_validation.cpp",
        "friendly_scored_validation.cpp",
        "robot_received_ball_validation.cpp",
    ],
    hdrs = [
        "ball_in_rectangle_validation.h",
        "friendly_scored_validation.h",
        "robot_received_ball_validation.h",
    ],
    deps = [
        "//software/ai/evaluation:robot",
        "//software/geom/algorithms",
        "//software/simulated_tests/validation:validation_function",
        "//software/world",
        "@boost//:coroutine2",
    ],
)
