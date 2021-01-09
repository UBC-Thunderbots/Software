package(default_visibility = ["//visibility:public"])

cc_library(
    name = "logger",
    hdrs = [
        "custom_logging_levels.h",
        "logger.h",
    ],
    deps = [
        ":coloured_cout_sink",
        "@g3log",
        "@g3sinks",
    ],
)

cc_test(
    name = "coloured_cout_sink_test",
    srcs = ["coloured_cout_sink_test.cpp"],
    deps = [
        ":coloured_cout_sink",
        ":logger",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "coloured_cout_sink",
    srcs = [
        "coloured_cout_sink.cpp",
    ],
    hdrs = [
        "coloured_cout_sink.h",
    ],
    deps = [
        "//software/util/make_enum",
        "@g3log",
    ],
)
