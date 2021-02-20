package(default_visibility = ["//visibility:public"])

cc_library(
    name = "non_terminating_validation_functions",
    testonly = True,
    srcs = [
        "robots_in_friendly_half_validation.cpp",
        "robots_not_in_center_circle_validation.cpp",
        "ball_in_play_validation.cpp"
    ],
    hdrs = [
        "robots_in_friendly_half_validation.h",
        "robots_not_in_center_circle_validation.h",
        "ball_in_play_validation.h",
    ],
    deps = [
        "//software/geom/algorithms",
        "//software/simulated_tests/validation:validation_function",
        "//software/world",
        "@boost//:coroutine2",
        "@gtest",
    ],
)
