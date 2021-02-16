package(default_visibility = ["//visibility:public"])

cc_library(
    name = "validation_functions",
    srcs = [
        "friendly_scored_validation.cpp",
        "robot_at_position_validation.cpp",
        "robot_received_ball_validation.cpp",
        "robots_in_center_circle_validation.cpp",
        "robots_in_friendly_half_validation.cpp",
    ],
    hdrs = [
        "friendly_scored_validation.h",
        "robot_at_position_validation.h",
        "robot_received_ball_validation.h",
        "robots_in_center_circle_validation.h",
        "robots_in_friendly_half_validation.h",
    ],
    deps = [
        "//software/geom/algorithms",
        "//software/simulated_tests/validation:validation_function",
        "//software/world",
        "@boost//:coroutine2",
        "@gtest",
    ],
)
