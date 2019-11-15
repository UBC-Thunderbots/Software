package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulated_test_fixture",
    srcs = ["simulated_test_fixture.cpp"],
    hdrs = ["simulated_test_fixture.h"],
    deps = [
        "//software/backend:simulator_backend",
        "//software/gui:visualizer_wrapper",
        "//software/util/logger",
        "//software/util/time:duration",
        "@gtest",
    ],
)

cc_test(
    name = "example_test",
    srcs = ["example_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        "//software/test_util",
        "//software/util/time:duration",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)
