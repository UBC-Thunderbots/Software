package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pass",
    srcs = ["pass.cpp"],
    hdrs = ["pass.h"],
    deps = [
        "//software/ai/hl/stp/evaluation:calc_best_shot",
        "//software/ai/world:robot",
    ],
)

cc_test(
    name = "pass_test",
    srcs = ["pass_test.cpp"],
    deps = [
        ":pass",
        "@gtest//:gtest_main",
    ],
)
