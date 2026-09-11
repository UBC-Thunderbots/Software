load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "make_enum",
    hdrs = [
        "make_enum.hpp",
        "make_enum_helpers.hpp",
    ],
)

cc_test(
    name = "make_enum_test",
    srcs = [":make_enum_test.cpp"],
    deps = [
        ":make_enum",
        "//shared/test_util:tbots_gtest_main",
    ],
)
