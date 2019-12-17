package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    deps = [
        ":closest_point",
        ":distance",
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
    name = "distance",
    srcs = [
        "distance.cpp",
    ],
    hdrs = [
        "distance.h",
    ],
    deps = [
        "//software/new_geom:angle",
        "//software/new_geom:geom_constants",
        "//software/new_geom:line",
        "//software/new_geom:point",
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
        "@gtest//:gtest_main",
    ],
)
