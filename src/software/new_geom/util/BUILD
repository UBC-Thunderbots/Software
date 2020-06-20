package(default_visibility = ["//visibility:public"])

cc_library(
    name = "algorithms",
    srcs = [
        "almost_equal.cpp",
        "closest_point.cpp",
        "collinear.cpp",
        "contains.cpp",
        "distance.cpp",
        # TODO: uncomment
        #"find_open_circles.cpp",
        "intersection.cpp",
        "intersects.cpp",
    ],
    hdrs = [
        "almost_equal.h",
        "closest_point.h",
        "collinear.h",
        "contains.h",
        "distance.h",
        # TODO: uncomment
        #"find_open_circles.h",
        "intersection.h",
        "intersects.h",
    ],
    deps = [
        "//software/logger",
        "//software/new_geom:angle",
        "//software/new_geom:circle",
        "//software/new_geom:geom_constants",
        "//software/new_geom:line",
        "//software/new_geom:point",
        "//software/new_geom:point_boost_geometry_compatability",
        "//software/new_geom:polygon",
        "//software/new_geom:ray",
        "//software/new_geom:rectangle",
        "//software/new_geom:segment",
        "//software/new_geom:vector",
    ],
)

cc_test(
    name = "almost_equal_test",
    srcs = [
        "almost_equal_test.cpp",
    ],
    deps = [
        ":algorithms",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "intersects_test",
    srcs = [
        "intersects_test.cpp",
    ],
    deps = [
        ":algorithms",
        "//software/new_geom:rectangle",
        "//software/new_geom:triangle",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "intersection_test",
    srcs = [
        "intersection_test.cpp",
    ],
    deps = [
        ":algorithms",
        "//software/new_geom:rectangle",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "collinear_test",
    srcs = [
        "collinear_test.cpp",
    ],
    deps = [
        ":algorithms",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "contains_test",
    srcs = [
        "contains_test.cpp",
    ],
    deps = [
        ":algorithms",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "closest_point_test",
    srcs = [
        "closest_point_test.cpp",
    ],
    deps = [
        ":closest_point",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "distance_test",
    srcs = [
        "distance_test.cpp",
    ],
    deps = [
        ":distance",
        "//software/new_geom:rectangle",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "find_open_circles_test",
    srcs = [
        "find_open_circles_test.cpp",
    ],
    deps = [
        ":find_open_circles",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

# TODO: uncomment
#cc_library(
#    name = "voronoi_diagram",
#    srcs = [
#        "voronoi_diagram.cpp",
#    ],
#    hdrs = [
#        "voronoi_diagram.h",
#    ],
#    deps = [
#        ":distance",
#        ":intersection",
#        "//software/logger",
#        "//software/new_geom:circle",
#        "//software/new_geom:point",
#        "//software/new_geom:point_boost_geometry_compatability",
#        "//software/new_geom:rectangle",
#    ],
#)
#
#cc_test(
#    name = "voronoi_diagram_test",
#    srcs = [
#        "voronoi_diagram_test.cpp",
#    ],
#    deps = [
#        ":voronoi_diagram",
#        "//software/test_util",
#        "@gtest//:gtest_main",
#    ],
#)
