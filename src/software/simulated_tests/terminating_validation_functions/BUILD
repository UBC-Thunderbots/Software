package(default_visibility = ["//visibility:public"])

cc_library(
    name = "terminating_validation_functions",
    testonly = True,
    srcs = [
        "ball_at_point_validation.cpp",
        "ball_kicked_validation.cpp",
        "friendly_scored_validation.cpp",
        "robot_halt_validation.cpp",
        "robot_in_center_circle_validation.cpp",
        "robot_in_polygon_validation.cpp",
        "robot_received_ball_validation.cpp",
        "robot_state_validation.cpp",
    ],
    hdrs = [
        "ball_at_point_validation.h",
        "ball_kicked_validation.h",
        "friendly_scored_validation.h",
        "robot_halt_validation.h",
        "robot_in_center_circle_validation.h",
        "robot_in_polygon_validation.h",
        "robot_received_ball_validation.h",
        "robot_state_validation.h",
    ],
    deps = [
        "//software/geom/algorithms",
        "//software/simulated_tests/validation:validation_function",
        "//software/world",
        "@boost//:coroutine2",
        "@gtest",
    ],
)
