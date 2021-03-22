package(default_visibility = ["//visibility:public"])

cc_library(
    name = "terminating_validation_functions",
    testonly = True,
    srcs = [
        "ball_kicked_validation.cpp",
        "friendly_scored_validation.cpp",
        "robot_at_position_validation.cpp",
        "robot_in_center_circle_validation.cpp",
        "robot_in_polygon_validation.cpp",
        "robot_is_at_orientation_validation.cpp",
        "robot_received_ball_validation.cpp",
    ],
    hdrs = [
        "ball_kicked_validation.h",
        "friendly_scored_validation.h",
        "robot_at_position_validation.h",
        "robot_in_center_circle_validation.h",
        "robot_in_polygon_validation.h",
        "robot_is_at_orientation_validation.h",
        "robot_received_ball_validation.h",
    ],
    deps = [
        "//software/geom/algorithms",
        "//software/simulated_tests/validation:validation_function",
        "//software/world",
        "@boost//:coroutine2",
        "@gtest",
    ],
)
