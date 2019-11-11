package(default_visibility = ["//visibility:public"])

cc_test(
    name = "integration_tests",
    srcs = ["simulated_test_fixture.cpp"],
    deps = [
        "//software/backend:simulator_backend",
        "//software/gui:visualizer_wrapper",
        "//software/test_util",
        "//software/util/logger",
        "//software/util/time:duration",
        "@gtest//:gtest_main",
    ],
)
