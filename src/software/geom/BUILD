package(default_visibility = ["//visibility:public"])

cc_library(
    name = "angle",
    hdrs = ["angle.h"],
    deps = [":geom_constants"],
)

cc_library(
    name = "angle_segment",
    srcs = ["angle_segment.cpp"],
    hdrs = ["angle_segment.h"],
    deps = [
        ":angle",
        "//software/logger",
    ],
)

cc_library(
    name = "angle_map",
    srcs = ["angle_map.cpp"],
    hdrs = ["angle_map.h"],
    deps = [":angle_segment"],
)

cc_library(
    name = "segment",
    srcs = ["segment.cpp"],
    hdrs = ["segment.h"],
    deps = [
        ":point",
        ":vector",
    ],
)

cc_library(
    name = "angular_velocity",
    hdrs = ["angular_velocity.h"],
    deps = [":angle"],
)

cc_library(
    name = "angular_acceleration",
    hdrs = ["angular_acceleration.h"],
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
    name = "polynomial1d",
    srcs = ["polynomial1d.cpp"],
    hdrs = ["polynomial1d.h"],
    deps = [
        ":geom_constants",
        "@eigen",
    ],
)

cc_library(
    name = "polynomial2d",
    srcs = ["polynomial2d.cpp"],
    hdrs = ["polynomial2d.h"],
    deps = [
        ":point",
        ":polynomial1d",
    ],
)

cc_library(
    name = "ray",
    srcs = ["ray.cpp"],
    hdrs = ["ray.h"],
    deps = [
        ":point",
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

cc_library(
    name = "spline2d",
    srcs = ["spline2d.cpp"],
    hdrs = ["spline2d.h"],
    deps = [
        ":point",
        ":polynomial2d",
    ],
)

cc_library(
    name = "linear_spline2d",
    srcs = ["linear_spline2d.cpp"],
    hdrs = ["linear_spline2d.h"],
    deps = [
        ":spline2d",
    ],
)

cc_library(
    name = "bezier_curve2d",
    srcs = ["bezier_curve2d.cpp"],
    hdrs = ["bezier_curve2d.h"],
    deps = [
        ":point",
        ":spline2d",
    ],
)

cc_library(
    name = "cubic_bezier_spline2d",
    srcs = ["cubic_bezier_spline2d.cpp"],
    hdrs = ["cubic_bezier_spline2d.h"],
    deps = [
        ":bezier_curve2d",
        ":spline2d",
    ],
)

cc_library(
    name = "stadium",
    srcs = ["stadium.cpp"],
    hdrs = ["stadium.h"],
    deps = [
        ":convex_shape",
        ":point",
        ":polygon",
        ":segment",
    ],
)

cc_test(
    name = "stadium_test",
    srcs = [
        "stadium_test.cpp",
    ],
    deps = [
        ":stadium",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "angle_test",
    srcs = [
        "angle_test.cpp",
    ],
    deps = [
        ":angle",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "angle_map_test",
    srcs = [
        "angle_map_test.cpp",
    ],
    deps = [
        ":angle_map",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "angle_segment_test",
    srcs = [
        "angle_segment_test.cpp",
    ],
    deps = [
        ":angle_segment",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "line_test",
    srcs = [
        "line_test.cpp",
    ],
    deps = [
        ":line",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "point_test",
    srcs = [
        "point_test.cpp",
    ],
    deps = [
        ":point",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "polynomial1d_test",
    srcs = [
        "polynomial1d_test.cpp",
    ],
    deps = [
        ":polynomial1d",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "polynomial2d_test",
    srcs = [
        "polynomial2d_test.cpp",
    ],
    deps = [
        ":polynomial2d",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "ray_test",
    srcs = [
        "ray_test.cpp",
    ],
    deps = [
        ":ray",
        "//shared/test_util:tbots_gtest_main",
        "//software/geom/algorithms",
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
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "circle_test",
    srcs = [
        "circle_test.cpp",
    ],
    deps = [
        ":circle",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "polygon_test",
    srcs = [
        "polygon_test.cpp",
    ],
    deps = [
        ":polygon",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "convex_polygon_test",
    srcs = [
        "convex_polygon_test.cpp",
    ],
    deps = [
        ":convex_polygon",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "rectangle_test",
    srcs = [
        "rectangle_test.cpp",
    ],
    deps = [
        ":rectangle",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "segment_test",
    srcs = [
        "segment_test.cpp",
    ],
    deps = [
        ":segment",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "triangle_test",
    srcs = [
        "triangle_test.cpp",
    ],
    deps = [
        ":triangle",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "linear_spline2d_test",
    srcs = [
        "linear_spline2d_test.cpp",
    ],
    deps = [
        ":linear_spline2d",
        "//shared/test_util:tbots_gtest_main",
        "//software/geom/algorithms",
        "//software/test_util",
    ],
)

cc_test(
    name = "bezier_curve2d_test",
    srcs = ["bezier_curve2d_test.cpp"],
    deps = [
        ":bezier_curve2d",
        "//shared/test_util:tbots_gtest_main",
        "//software/geom/algorithms",
        "//software/test_util",
    ],
)

cc_test(
    name = "cubic_bezier_spline2d_test",
    srcs = ["cubic_bezier_spline2d_test.cpp"],
    deps = [
        ":cubic_bezier_spline2d",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
