package(default_visibility = ["//visibility:public"])

cc_library(
    name = "logger",
    srcs = ["logger.c"],
    hdrs = ["logger.h"],
    deps = ["//shared/proto:tbots_nanopb_proto"],
)

cc_test(
    name = "logger_test",
    srcs = ["logger_test.cpp"],
    deps = [
        ":logger",
        "//shared/proto:tbots_nanopb_proto",
        "//shared/test_util:tbots_gtest_main",
    ],
)
