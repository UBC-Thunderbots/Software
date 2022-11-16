package(default_visibility = ["//visibility:public"])

cc_library(
    name = "algorithms",
    srcs = [
        "almost_equal.cpp",
        "calculate_block_cone.cpp",
        "closest_point.cpp",
        "collinear.cpp",
        "contains.cpp",
        "convex_angle.cpp",
        "distance.cpp",
        "find_open_circles.cpp",
        "furthest_point.cpp",
        "generate_velocity_obstacle.cpp",
        "intersection.cpp",
        "intersects.cpp",
        "rasterize.cpp",
        "voronoi_diagram.cpp",
    ],
    hdrs = [
        "almost_equal.h",
        "calculate_block_cone.h",
        "closest_point.h",
        "collinear.h",
        "contains.h",
        "convex_angle.h",
        "distance.h",
        "find_open_circles.h",
        "furthest_point.h",
        "generate_velocity_obstacle.h",
        "intersection.h",
        "intersects.h",
        "rasterize.h",
        "voronoi_diagram.h",
    ],
    deps = [
        "//software/ai/navigator/path_planner/hrvo:velocity_obstacle",
        "//software/geom:angle",
        "//software/geom:circle",
        "//software/geom:geom_constants",
        "//software/geom:line",
        "//software/geom:point",
        "//software/geom:point_boost_geometry_compatability",
        "//software/geom:polygon",
        "//software/geom:ray",
        "//software/geom:rectangle",
        "//software/geom:segment",
        "//software/geom:vector",
        "//software/logger",
    ],
)

cc_test(
    name = "almost_equal_test",
    srcs = [
        "almost_equal_test.cpp",
    ],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "intersects_test",
    srcs = [
        "intersects_test.cpp",
    ],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
        "//software/geom:rectangle",
        "//software/geom:triangle",
    ],
)

cc_test(
    name = "intersection_test",
    srcs = [
        "intersection_test.cpp",
    ],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
        "//software/geom:rectangle",
        "//software/test_util",
    ],
)

cc_test(
    name = "collinear_test",
    srcs = [
        "collinear_test.cpp",
    ],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "contains_test",
    srcs = [
        "contains_test.cpp",
    ],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "closest_point_test",
    srcs = [
        "closest_point_test.cpp",
    ],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "distance_test",
    srcs = [
        "distance_test.cpp",
    ],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
        "//software/geom:rectangle",
    ],
)

cc_test(
    name = "find_open_circles_test",
    srcs = [
        "find_open_circles_test.cpp",
    ],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "voronoi_diagram_test",
    srcs = [
        "voronoi_diagram_test.cpp",
    ],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "furthest_point_test",
    srcs = ["furthest_point_test.cpp"],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "calculate_block_cone_test",
    srcs = ["calculate_block_cone_test.cpp"],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "convex_angle_test",
    srcs = ["convex_angle_test.cpp"],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "rasterize_test",
    srcs = ["rasterize_test.cpp"],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
