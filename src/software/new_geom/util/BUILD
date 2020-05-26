package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    deps = [
        ":closest_point",
        ":distance",
        ":intersects",
    ],
)

cc_library(
    name = "closest_point",
    srcs = [
        "closest_point.cpp",
    ],
    hdrs = [
        "closest_point.h",
    ],
    deps = [
        "//software/new_geom:angle",
        "//software/new_geom:geom_constants",
        "//software/new_geom:line",
        "//software/new_geom:point",
        "//software/new_geom:vector",
    ],
)

cc_library(
    name = "almost_equal",
    srcs = [
        "almost_equal.cpp",
    ],
    hdrs = [
        "almost_equal.h",
    ],
    deps = [
        "//software/new_geom:geom_constants",
    ],
)

cc_test(
    name = "almost_equal_test",
    srcs = [
        "almost_equal_test.cpp",
    ],
    deps = [
        ":almost_equal",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "intersects",
    srcs = [
        "intersects.cpp",
    ],
    hdrs = [
        "intersects.h",
    ],
    deps = [
        ":contains",
        ":distance",
        ":intersection",
        "//software/new_geom:circle",
        "//software/new_geom:line",
        "//software/new_geom:point_boost_geometry_compatability",
        "//software/new_geom:polygon",
        "//software/new_geom:ray",
        "//software/new_geom:segment",
    ],
)

cc_test(
    name = "intersects_test",
    srcs = [
        "intersects_test.cpp",
    ],
    deps = [
        ":intersects",
        "//software/new_geom:rectangle",
        "//software/new_geom:triangle",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "intersection",
    srcs = [
        "intersection.cpp",
    ],
    hdrs = [
        "intersection.h",
    ],
    deps = [
        ":almost_equal",
        ":collinear",
        "//software/new_geom:line",
        "//software/new_geom:point_boost_geometry_compatability",
        "//software/new_geom:polygon",
        "//software/new_geom:ray",
        "//software/new_geom:segment",
    ],
)

cc_test(
    name = "intersection_test",
    srcs = [
        "intersection_test.cpp",
    ],
    deps = [
        ":intersection",
        "//software/new_geom:rectangle",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "collinear",
    srcs = [
        "collinear.cpp",
    ],
    hdrs = [
        "collinear.h",
    ],
    deps = [
        ":almost_equal",
        "//software/new_geom:point",
    ],
)

cc_test(
    name = "collinear_test",
    srcs = [
        "collinear_test.cpp",
    ],
    deps = [
        ":collinear",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "contains",
    srcs = [
        "contains.cpp",
    ],
    hdrs = [
        "contains.h",
    ],
    deps = [
        ":distance",
        "//software/new_geom:circle",
        "//software/new_geom:segment",
    ],
)

cc_test(
    name = "contains_test",
    srcs = [
        "contains_test.cpp",
    ],
    deps = [
        ":contains",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "distance",
    srcs = [
        "distance.cpp",
    ],
    hdrs = [
        "distance.h",
    ],
    deps = [
        "//software/new_geom:angle",
        "//software/new_geom:circle",
        "//software/new_geom:geom_constants",
        "//software/new_geom:line",
        "//software/new_geom:point",
        "//software/new_geom:point_boost_geometry_compatability",
        "//software/new_geom:polygon",
        "//software/new_geom:segment",
        "//software/new_geom:vector",
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
