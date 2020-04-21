package(default_visibility = ["//visibility:public"])

cc_library(
    name = "printable_enum_macro",
    hdrs = ["printable_enum_macro.h"],
)

cc_test(
    name = "printable_enum_macro_test",
    srcs = [":printable_enum_macro_test.cpp"],
    deps = [
        ":printable_enum_macro",
        "@gtest//:gtest_main",
    ],
)
