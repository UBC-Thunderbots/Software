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
    name = "physics_ball",
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
    name = "physics_ball_test",
    srcs = ["physics_ball_test.cpp"],
    deps = [
        ":physics_ball",
        "//software/world:ball",
        "@box2d",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "physics_field",
    srcs = ["physics_field.cpp"],
    hdrs = ["physics_field.h"],
    deps = [
        ":box2d_util",
        "//software/util/time:timestamp",
        "//software/world:field",
        "@box2d",
    ],
)

cc_test(
    name = "physics_field_test",
    srcs = ["physics_field_test.cpp"],
    deps = [
        ":physics_ball",
        ":physics_field",
        "//software/test_util",
        "//software/world:field",
        "@box2d",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "physics_simulator",
    srcs = ["physics_simulator.cpp"],
    hdrs = ["physics_simulator.h"],
    deps = [
        ":physics_ball",
        ":physics_field",
        "//software/util/time:duration",
        "//software/util/time:timestamp",
        "//software/world",
        "@box2d",
    ],
)

cc_test(
    name = "physics_simulator_test",
    srcs = ["physics_simulator_test.cpp"],
    deps = [
        ":physics_simulator",
        "//software/test_util",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)
