package(default_visibility = ["//visibility:public"])

cc_library(
    name = "all",
    srcs = [
        "polygon.cpp",
        "rectangle.cpp",
        "util.cpp",
    ],
    hdrs = [
        "angle.h",
        "circle.h",
        "line.h",
        "point.h",
        "polygon.h",
        "ray.h",
        "rectangle.h",
        "segment.h",
        "util.h",
    ],
    copts = [
        "-std=c++17",
    ],
    deps = [
        "@boost//:geometry",
        "@g3log//:all",
    ],
)

cc_test(
    name = "angle_test",
    srcs = [
        "angle_test.cpp",
    ],
    deps = [
        ":all",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "point_test",
    srcs = [
        "point_test.cpp",
    ],
    deps = [
        ":all",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "polygon_test",
    srcs = [
        "polygon_test.cpp",
    ],
    deps = [
        ":all",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "util_test",
    srcs = [
        "util_test.cpp",
    ],
    deps = [
        ":all",
        "@gtest//:gtest_main",
    ],
)
