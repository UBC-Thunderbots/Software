package(default_visibility = ["//visibility:public"])

# TODO (Issue #790): remove all `strip_include_prefix` once firmware has been setup to have includes
#       from top-level

cc_library(
    name = "tbots_math",
    hdrs = ["tbots_math.h"],
    strip_include_prefix = "/firmware/main",
)

cc_library(
    name = "matrix",
    hdrs = ["matrix.h"],
    strip_include_prefix = "/firmware/main",
)

cc_library(
    name = "polynomial_1d",
    srcs = ["polynomial_1d.c"],
    hdrs = ["polynomial_1d.h"],
    deps = [],
)

cc_test(
    name = "polynomial_1d_test",
    srcs = ["polynomial_1d_test.cpp"],
    deps = [
        ":polynomial_1d",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "polynomial_2d",
    srcs = ["polynomial_2d.c"],
    hdrs = ["polynomial_2d.h"],
    deps = [
        ":polynomial_1d",
        ":vector_2d",
    ],
)

cc_test(
    name = "polynomial_2d_test",
    srcs = ["polynomial_2d_test.cpp"],
    deps = [
        ":polynomial_2d",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "vector_2d",
    hdrs = ["vector_2d.h"],
)
