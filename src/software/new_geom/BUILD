package(default_visibility = ["//visibility:public"])

cc_library(
    name = "new_geom",
    srcs = [],
    hdrs = [
        "ray.h",
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
    name = "ray_test",
    srcs = [
        "ray_test.cpp",
    ],
    deps = [
        ":new_geom",
        "@gtest//:gtest_main",
    ],
)
