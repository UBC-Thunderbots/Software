package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gradient_descent",
    hdrs = [
        "gradient_descent_optimizer.hpp",
    ],
)

cc_test(
    name = "gradient_descent_test",
    srcs = ["gradient_descent_test.cpp"],
    deps = [
        ":gradient_descent",
        "//shared/test_util:tbots_gtest_main",
    ],
)
