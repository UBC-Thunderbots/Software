package(default_visibility = ["//visibility:public"])

cc_library(
    name = "new_geom",
    srcs = [
        "circle.cpp",
        "polynomial.cpp",
    ],
    hdrs = [
        "angular_velocity.h",
        "circle.h",
        "convex_shape.h",
        "polynomial.h",
        "ray.h",
        "shape.h",
    ],
    deps = [
        ":angle",
        ":closest_point",
        ":distance",
        ":geom_constants",
        ":line",
        ":point",
        ":vector",
    ],
)

cc_library(
    name = "angle",
    hdrs = ["angle.h"],
)

cc_library(
    name = "closest_point",
    srcs = [
        "util/closest_point.cpp",
    ],
    hdrs = [
        "util/closest_point.h",
    ],
    deps = [
        ":angle",
        ":geom_constants",
        ":line",
        ":point",
        ":vector",
    ],
)

cc_library(
    name = "distance",
    srcs = [
        "util/distance.cpp",
    ],
    hdrs = [
        "util/distance.h",
    ],
    deps = [
        ":angle",
        ":geom_constants",
        ":line",
        ":point",
        ":vector",
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
        "@boost//:geometry",
        "@boost//:polygon",
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
        ":new_geom",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "closest_point_test",
    srcs = [
        "util/closest_point_test.cpp",
    ],
    deps = [
        ":closest_point",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "distance_test",
    srcs = [
        "util/distance_test.cpp",
    ],
    deps = [
        ":distance",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "line_test",
    srcs = [
        "line_test.cpp",
    ],
    deps = [
        ":new_geom",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "point_test",
    srcs = [
        "point_test.cpp",
    ],
    deps = [
        ":new_geom",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "polynomial_test",
    srcs = [
        "polynomial_test.cpp",
    ],
    deps = [
        ":new_geom",
        "@gtest//:gtest_main",
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

cc_test(
    name = "vector_test",
    srcs = [
        "vector_test.cpp",
    ],
    deps = [
        ":new_geom",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "circle_test",
    srcs = [
        "circle_test.cpp",
    ],
    deps = [
        ":new_geom",
        "@gtest//:gtest_main",
    ],
)
