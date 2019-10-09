package(default_visibility = ["//visibility:public"])

cc_library(
    name = "box2d_util",
    srcs = ["box2d_util.cpp"],
    hdrs = ["box2d_util.h"],
    deps = [
        "@box2d",
    ],
)

cc_test(
    name = "box2d_util_test",
    srcs = ["box2d_util_test.cpp"],
    deps = [
        ":box2d_util",
        "@box2d",
        "@gtest//:gtest_main",
    ],
)
