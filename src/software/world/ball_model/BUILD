package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ball_model",
    hdrs = ["ball_model.h"],
    deps = [
        "//software/time:duration",
        "//software/world:ball_state",
    ],
)

cc_library(
    name = "two_stage_linear_ball_model",
    srcs = ["two_stage_linear_ball_model.cpp"],
    hdrs = ["two_stage_linear_ball_model.h"],
    deps = [
        ":ball_model",
        "//software/physics",
    ],
)

cc_test(
    name = "two_stage_linear_ball_model_test",
    srcs = ["two_stage_linear_ball_model_test.cpp"],
    deps = [
        ":two_stage_linear_ball_model",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
