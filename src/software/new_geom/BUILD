package(default_visibility = ["//visibility:public"])

cc_library(
    name = "new_geom",
    srcs = [
        "circle.cpp",
        "line.cpp",
        "point.cpp",
        "polynomial.cpp",
        "vector.cpp",
    ],
    hdrs = [
        "angle.h",
        "angular_velocity.h",
        "circle.h",
        "convex_shape.h",
        "geom_constants.h",
        "line.h",
        "point.h",
        "polynomial.h",
        "ray.h",
        "shape.h",
        "vector.h",
    ],
    deps = [
        ":closest_point",
        ":distance",
        "@boost//:geometry",
        "@boost//:polygon",
    ],
)

cc_library(
    name = "closest_point",
    srcs = [
        "line.cpp",
        "point.cpp",
        "util/closest_point.cpp",
        "vector.cpp",
    ],
    hdrs = [
        "angle.h",
        "geom_constants.h",
        "line.h",
        "point.h",
        "util/closest_point.h",
        "vector.h",
    ],
    deps = [
        "@boost//:geometry",
        "@boost//:polygon",
    ],
)

cc_library(
    name = "distance",
    srcs = [
        "line.cpp",
        "point.cpp",
        "util/distance.cpp",
        "vector.cpp",
    ],
    hdrs = [
        "angle.h",
        "geom_constants.h",
        "line.h",
        "point.h",
        "util/distance.h",
        "vector.h",
    ],
    deps = [
        "@boost//:geometry",
        "@boost//:polygon",
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
