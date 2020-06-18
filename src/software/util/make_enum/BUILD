package(
    default_visibility = ["//visibility:public"],
    features = ["warnings_as_errors"],
)

cc_library(
    name = "make_enum",
    hdrs = ["make_enum.h"],
)

cc_test(
    name = "make_enum_test",
    srcs = [":make_enum_test.cpp"],
    deps = [
        ":make_enum",
        "@gtest//:gtest_main",
    ],
)
