package(default_visibility = ["//visibility:public"])

cc_library(
    name = "logger",
    hdrs = [
        "custom_logging_levels.h",
        "logger.h",
    ],
    deps = [
        "@g3log",
        "@g3sinks",
    ],
)

cc_test(
    name = "logger_test",
    srcs = ["logger_test.cpp"],
    deps = [
        ":logger",
        "@gtest//:gtest_main",
    ],
)
