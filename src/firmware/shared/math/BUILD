package(default_visibility = ["//visibility:public"])

cc_library(
    name = "matrix",
    srcs = ["matrix.c"],
    hdrs = ["matrix.h"],
    deps = [],
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
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "matrix_test",
    srcs = ["matrix_test.cpp"],
    deps = [
        ":matrix",
        "//shared/test_util:tbots_gtest_main",
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
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "vector_2d",
    hdrs = ["vector_2d.h"],
)

cc_library(
    name = "tbots_math",
    srcs = ["tbots_math.c"],
    hdrs = ["tbots_math.h"],
    deps = [],
)

cc_test(
    name = "tbots_math_test",
    srcs = ["tbots_math_test.cpp"],
    deps = [
        ":tbots_math",
        "//shared/test_util:tbots_gtest_main",
    ],
)
