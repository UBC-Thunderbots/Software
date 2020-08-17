package(default_visibility = ["//visibility:public"])

cc_library(
    name = "typename",
    srcs = ["typename.cpp"],
    hdrs = ["typename.h"],
)

cc_test(
    name = "typename_test",
    srcs = [":typename_test.cpp"],
    deps = [
        ":typename",
        "@gtest//:gtest_main",
    ],
)
