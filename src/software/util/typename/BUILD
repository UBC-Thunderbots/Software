load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

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
        "//shared/test_util:tbots_gtest_main",
    ],
)
