package(default_visibility = ["//visibility:public"])

cc_library(
    name = "test_util",
    testonly = True,
    srcs = ["test_util.cpp"],
    hdrs = ["test_util.h"],
    deps = ["//software/world"],
)

cc_test(
    name = "test_util_test",
    srcs = ["test_util_test.cpp"],
    deps = [
        ":test_util",
        "@gtest//:gtest_main",
    ],
)
