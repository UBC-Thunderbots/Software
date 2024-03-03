package(default_visibility = ["//visibility:public"])

cc_library(
    name = "algorithms",
    srcs = [
        "almost_equal.cpp",
        "axis_aligned_bounding_box.cpp",
        "calculate_block_cone.cpp",
        "closest_point.cpp",
        "collinear.cpp",
        "contains.cpp",
        "convex_angle.cpp",
        "distance.cpp",
        "find_open_circles.cpp",
        "furthest_point.cpp",
        "intersection.cpp",
        "intersects.cpp",
        "rasterize.cpp",
        "step_along_perimeter.cpp",
        "voronoi_diagram.cpp",
    ],
    hdrs = [
        "almost_equal.h",
        "axis_aligned_bounding_box.h",
        "calculate_block_cone.h",
        "closest_point.h",
        "collinear.h",
        "contains.h",
        "convex_angle.h",
        "distance.h",
        "find_open_circles.h",
        "furthest_point.h",
        "intersection.h",
        "intersects.h",
        "rasterize.h",
        "step_along_perimeter.h",
        "voronoi_diagram.h",
    ],
    deps = [
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
        "//software/geom:stadium",
        "//software/geom:vector",
        "//software/logger",
    ],
)

cc_library(
    name = "end_in_obstacle_sample",
    srcs = ["end_in_obstacle_sample.cpp"],
    hdrs = ["end_in_obstacle_sample.h"],
    deps = [
        "//software/ai/navigator/obstacle",
    ],
)

cc_library(
    name = "nearest_neighbor_search",
    hdrs = [
        "nearest_neighbor_search.hpp",
    ],
    deps = [],
)

cc_library(
    name = "is_in_range",
    srcs = [
        "is_in_range.cpp",
    ],
    hdrs = [
        "is_in_range.h",
    ],
    deps = [],
)

cc_test(
    name = "nearest_neighbor_test",
    srcs = ["nearest_neighbor_test.cpp"],
    deps = [
        ":nearest_neighbor_search",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
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
    name = "axis_aligned_bounding_box_test",
    srcs = [
        "axis_aligned_bounding_box_test.cpp",
    ],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
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

cc_test(
    name = "step_along_perimeter_test",
    srcs = ["step_along_perimeter_test.cpp"],
    deps = [
        ":algorithms",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "end_in_obstacle_sample_test",
    srcs = ["end_in_obstacle_sample_test.cpp"],
    deps = [
        ":algorithms",
        ":end_in_obstacle_sample",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/navigator/obstacle:robot_navigation_obstacle_factory",
        "//software/test_util",
    ],
)
