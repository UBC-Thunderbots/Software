package(default_visibility = ["//visibility:public"])

cc_library(
    name = "angle",
    hdrs = ["angle.h"],
    deps = [":geom_constants"],
)

cc_library(
    name = "segment",
    hdrs = ["segment.h"],
    deps = [
        ":point",
        "//software/new_geom/util:almost_equal",
        "//software/new_geom/util:collinear",
    ],
)

cc_library(
    name = "angular_velocity",
    hdrs = ["angular_velocity.h"],
    deps = [":angle"],
)

cc_library(
    name = "shape",
    hdrs = ["shape.h"],
    deps = [":point"],
)

cc_library(
    name = "convex_shape",
    hdrs = ["convex_shape.h"],
    deps = [":shape"],
)

cc_library(
    name = "polygon",
    srcs = ["polygon.cpp"],
    hdrs = ["polygon.h"],
    deps = [
        ":segment",
        ":shape",
    ],
)

cc_library(
    name = "convex_polygon",
    srcs = ["convex_polygon.cpp"],
    hdrs = ["convex_polygon.h"],
    deps = [
        ":convex_shape",
        ":polygon",
    ],
)

cc_library(
    name = "circle",
    srcs = ["circle.cpp"],
    hdrs = ["circle.h"],
    deps = [
        ":convex_shape",
        ":point",
    ],
)

cc_library(
    name = "rectangle",
    srcs = ["rectangle.cpp"],
    hdrs = ["rectangle.h"],
    deps = [
        ":convex_polygon",
    ],
)

cc_library(
    name = "triangle",
    srcs = ["triangle.cpp"],
    hdrs = ["triangle.h"],
    deps = [
        ":convex_polygon",
    ],
)

cc_library(
    name = "geom_constants",
    hdrs = ["geom_constants.h"],
)

cc_library(
    name = "line",
    srcs = ["line.cpp"],
    hdrs = ["line.h"],
    deps = [
        ":angle",
        ":point",
        ":vector",
    ],
)

cc_library(
    name = "point",
    srcs = ["point.cpp"],
    hdrs = ["point.h"],
    deps = [
        ":angle",
        ":geom_constants",
        ":vector",
    ],
)

cc_library(
    name = "point_boost_geometry_compatability",
    hdrs = ["point_boost_geometry_compatability.h"],
    deps = [
        ":point",
        "@boost//:geometry",
        "@boost//:polygon",
    ],
)

cc_library(
    name = "polynomial",
    srcs = ["polynomial.cpp"],
    hdrs = ["polynomial.h"],
    deps = [":geom_constants"],
)

cc_library(
    name = "ray",
    srcs = ["ray.cpp"],
    hdrs = ["ray.h"],
    deps = [
        ":point",
        "//software/new_geom/util:collinear",
    ],
)

cc_library(
    name = "vector",
    srcs = ["vector.cpp"],
    hdrs = ["vector.h"],
    deps = [
        "angle",
        "geom_constants",
        "@boost//:geometry",
    ],
)

cc_test(
    name = "angle_test",
    srcs = [
        "angle_test.cpp",
    ],
    deps = [
        ":angle",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "line_test",
    srcs = [
        "line_test.cpp",
    ],
    deps = [
        ":line",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "point_test",
    srcs = [
        "point_test.cpp",
    ],
    deps = [
        ":point",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "polynomial_test",
    srcs = [
        "polynomial_test.cpp",
    ],
    deps = [
        ":polynomial",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "ray_test",
    srcs = [
        "ray_test.cpp",
    ],
    deps = [
        ":ray",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "segment_test",
    srcs = [
        "segment_test.cpp",
    ],
    deps = [
        ":segment",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "vector_test",
    srcs = [
        "vector_test.cpp",
    ],
    deps = [
        ":point",
        ":vector",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "circle_test",
    srcs = [
        "circle_test.cpp",
    ],
    deps = [
        ":circle",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "polygon_test",
    srcs = [
        "polygon_test.cpp",
    ],
    deps = [
        ":polygon",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "convex_polygon_test",
    srcs = [
        "convex_polygon_test.cpp",
    ],
    deps = [
        ":convex_polygon",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "rectangle_test",
    srcs = [
        "rectangle_test.cpp",
    ],
    deps = [
        ":rectangle",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "triangle_test",
    srcs = [
        "triangle_test.cpp",
    ],
    deps = [
        ":triangle",
        "@gtest//:gtest_main",
    ],
)
