package(default_visibility = ["//visibility:public"])

cc_library(
    name = "make_enum_macro",
    hdrs = ["make_enum_macro.h"],
)

cc_test(
    name = "make_enum_macro_test",
    srcs = [":make_enum_macro_test.cpp"],
    deps = [
        ":make_enum_macro",
        "@gtest//:gtest_main",
    ],
)
