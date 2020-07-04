package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ball_model",
    srcs = ["ball_model.cpp"],
    hdrs = ["ball_model.h"],
    deps = [
        "//software/time:duration",
        "//software/world:ball_state",
    ],
)

cc_library(
    name = "linear_ball_model",
    srcs = ["linear_ball_model.cpp"],
    hdrs = ["linear_ball_model.h"],
    deps = [
        ":ball_model",
    ],
)

cc_library(
    name = "linear_ball_model_test",
    srcs = ["linear_ball_model_test.cpp"],
    deps = [
        ":linear_ball_model",
        "@gtest//:gtest_main",
    ],
)
