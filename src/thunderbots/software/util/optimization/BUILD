package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gradient_descent",
    hdrs = [
        "gradient_descent.h",
        "gradient_descent.tpp",
    ],
)

cc_test(
    name = "gradient_descent_test",
    srcs = ["gradient_descent_test.cpp"],
    deps = [
        ":gradient_descent",
        "@gtest//:gtest_main",
    ],
)
