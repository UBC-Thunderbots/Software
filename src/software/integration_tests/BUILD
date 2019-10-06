package(default_visibility = ["//visibility:public"])

cc_test(
    name = "integration_tests",
    srcs = ["integration_test_fixture.cpp"],
    deps = [
        "//software:typedefs",
        "//software/ai:ai_wrapper",
        "//software/backend:simulator_backend",
        "//software/gui:visualizer_wrapper",
        "//software/util/logger",
        "@g3log",
        "@gtest//:gtest_main",
    ],
)
