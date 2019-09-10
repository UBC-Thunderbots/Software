cc_library(
    name = "geom",
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
    copts = ["-std=c++17"],
    deps = [],
)

cc_test(
    name = "angle_test",
    srcs = [
        "angle_test.cpp",
    ],
    deps = [
        "@gtest//:main",
    ],
)
