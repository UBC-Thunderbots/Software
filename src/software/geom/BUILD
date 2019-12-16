package(default_visibility = ["//visibility:public"])

cc_library(
    name = "geom",
    srcs = [
        "polygon.cpp",
        "polynomial.cpp",
        "rectangle.cpp",
        "shot.cpp",
        "spline.cpp",
        "util.cpp",
        "voronoi_diagram.cpp",
    ],
    hdrs = [
        "circle.h",
        "polygon.h",
        "polynomial.h",
        "rectangle.h",
        "segment.h",
        "shot.h",
        "spline.h",
        "util.h",
        "voronoi_diagram.h",
    ],
    copts = [
        "-std=c++17",
    ],
    deps = [
        "//software/new_geom",
        "@boost//:geometry",
        "@boost//:polygon",
        "@g3log",
    ],
)

cc_test(
    name = "spline_test",
    srcs = [
        "spline_test.cpp",
    ],
    deps = [
        ":geom",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "polynomial_test",
    srcs = [
        "polynomial_test.cpp",
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
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "voronoi_util_test",
    srcs = [
        "voronoi_diagram_test.cpp",
    ],
    deps = [
        ":geom",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
