package(default_visibility = ["//visibility:public"])

cc_library(
    name = "new_geom",
    srcs = [
        "circle.cpp",
        "convex_polygon.cpp",
        "polygon.cpp",
        "rectangle.cpp",
    ],
    hdrs = [
        "circle.h",
        "convex_polygon.h",
        "convex_shape.h",
        "polygon.h",
        "rectangle.h",
        "segment.h",
        "shape.h",
    ],
    deps = [
        ":angle",
        ":angular_velocity",
        ":geom_constants",
        ":line",
        ":point",
        ":polynomial",
        ":ray",
        ":vector",
        "//software/new_geom/util",
    ],
)

cc_library(
    name = "angle",
    hdrs = ["angle.h"],
)

cc_library(
    name = "angular_velocity",
    hdrs = ["angular_velocity.h"],
    deps = [":angle"],
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
    name = "polynomial",
    srcs = ["polynomial.cpp"],
    hdrs = ["polynomial.h"],
    deps = [":geom_constants"],
)

cc_library(
    name = "ray",
    srcs = ["ray.cpp"],
    hdrs = ["ray.h"],
    deps = [":point"],
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
    name = "convex_polygon_test",
    srcs = [
        "convex_polygon_test.cpp",
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
