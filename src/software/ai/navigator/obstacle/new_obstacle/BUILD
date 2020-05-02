package(default_visibility = ["//software/ai/navigator:__subpackages__"])

cc_library(
    name = "obstacle",
    srcs = [
        "circle_obstacle.cpp",
        "obstacle.cpp",
        "polygon_obstacle.cpp",
    ],
    hdrs = [
        "circle_obstacle.h",
        "obstacle.h",
        "polygon_obstacle.h",
    ],
    deps = [
        "//shared:constants",
        "//software/new_geom:circle",
        "//software/new_geom:point",
        "//software/new_geom:polygon",
        "//software/new_geom:segment",
        "//software/new_geom/util",
    ],
)

cc_test(
    name = "obstacle_test",
    srcs = ["obstacle_test.cpp"],
    deps = [
        ":obstacle",
        "//software/new_geom:point",
        "//software/new_geom:rectangle",
        "@gtest//:gtest_main",
    ],
)
