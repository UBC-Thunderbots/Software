package(default_visibility = ["//visibility:public"])

cc_library(
    name = "new_geom",
    srcs = [
        "circle.cpp",
        "point.cpp",
        "polygon.cpp",
        "polynomial.cpp",
        "rectangle.cpp",
        "vector.cpp",
    ],
    hdrs = [
        "angle.h",
        "angular_velocity.h",
        "circle.h",
        "convex_polygon.h",
        "convex_shape.h",
        "geom_constants.h",
        "point.h",
        "polygon.h",
        "polynomial.h",
        "ray.h",
        "rectangle.h",
        "segment.h",
        "shape.h",
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

cc_test(
    name = "polygon_test",
    srcs = [
        "polygon_test.cpp",
    ],
    deps = [
        ":new_geom",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "rectangle_test",
    srcs = [
        "rectangle_test.cpp",
    ],
    deps = [
        ":new_geom",
        "@gtest//:gtest_main",
    ],
)
