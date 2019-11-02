package(default_visibility = ["//software/ai/navigator:__subpackages__"])

cc_library(
    name = "obstacle",
    srcs = ["obstacle.cpp"],
    hdrs = ["obstacle.h"],
    deps = [
        "//shared:constants",
        "//software/geom",
        "//software/world:ball",
        "//software/world:robot",
    ],
)

cc_test(
    name = "obstacle_test",
    srcs = ["obstacle_test.cpp"],
    deps = [
        ":obstacle",
        "@gtest//:gtest_main",
    ],
)
