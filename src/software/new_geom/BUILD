package(default_visibility = ["//visibility:public"])

cc_library(
    name = "new_geom",
    srcs = [
        "polynomial.cpp",
    ],
    hdrs = [
        "polynomial.h",
    ],
    copts = [
        "-std=c++17",
    ],
    deps = [
        "//software/geom",
        "@boost//:geometry",
        "@boost//:polygon",
    ],
)

cc_test(
    name = "polynomial_test",
    srcs = [
        "polynomial_test.cpp",
    ],
    deps = [
        ":new_geom",
        "@gtest//:gtest_main",
    ],
)
