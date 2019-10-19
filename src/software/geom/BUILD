package(default_visibility = ["//visibility:public"])

cc_library(
    name = "geom",
    srcs = [
        "polygon.cpp",
        "rectangle.cpp",
        "shot.cpp",
        "util.cpp",
        "polynomial.cpp",
        "spline.cpp",
    ],
    hdrs = [
        "angle.h",
        "circle.h",
        "line.h",
        "point.h",
        "polygon.h",
        "ray.h",
        "rectangle.h",
        "segment.h",
        "shot.h",
        "util.h",
        "polynomial.h",
        "spline.h",
    ],
    copts = [
        "-std=c++17",
    ],
    deps = [
        "@boost//:geometry",
        "@g3log",
        "@boost//:polygon",
    ],
)

cc_library(
    name = "spline",
    srcs = [
        "polynomial.cpp",
        "spline.cpp",
    ],
    hdrs = [
        "polynomial.h",
        "spline.h",
    ],
    copts = [
        "-std=c++17",
    ],
    deps = [
        "@boost//:geometry",
        "@g3log",
        "@boost//:polygon",
        ":geom"
    ],
)

cc_test(
    name = "angle_test",
    srcs = [
        "angle_test.cpp",
    ],
    deps = [
        ":geom",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "point_test",
    srcs = [
        "point_test.cpp",
    ],
    deps = [
        ":geom",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "polygon_test",
    srcs = [
        "polygon_test.cpp",
    ],
    deps = [
        ":geom",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "util_test",
    srcs = [
        "util_test.cpp",
    ],
    deps = [
        ":geom",
        "@gtest//:gtest_main",
    ],
)
