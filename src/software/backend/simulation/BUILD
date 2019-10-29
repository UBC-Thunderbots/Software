package(default_visibility = ["//visibility:public"])

cc_library(
    name = "box2d_util",
    srcs = ["box2d_util.cpp"],
    hdrs = ["box2d_util.h"],
    deps = [
        "//software/geom",
        "@box2d",
    ],
)

cc_test(
    name = "box2d_util_test",
    srcs = ["box2d_util_test.cpp"],
    deps = [
        ":box2d_util",
        "//software/geom",
        "@box2d",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "ball",
    srcs = ["physics_ball.cpp"],
    hdrs = ["physics_ball.h"],
    deps = [
        ":box2d_util",
        "//shared:constants",
        "//software/geom",
        "//software/util/time:timestamp",
        "//software/world:ball",
        "@box2d",
    ],
)

cc_test(
    name = "ball_test",
    srcs = ["physics_ball_test.cpp"],
    deps = [
        ":ball",
        "//software/world:ball",
        "@box2d",
        "@gtest//:gtest_main",
    ],
)
