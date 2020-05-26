package(default_visibility = ["//visibility:public"])

cc_library(
    name = "catch_primitive",
    srcs = ["catch_primitive.cpp"],
    hdrs = ["catch_primitive.h"],
    deps = [
        ":primitive",
        "//software/geom",
    ],
)

cc_test(
    name = "catch_primitive_test",
    srcs = ["catch_primitive_test.cpp"],
    deps = [
        ":catch_primitive",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "chip_primitive",
    srcs = ["chip_primitive.cpp"],
    hdrs = ["chip_primitive.h"],
    deps = [
        ":primitive",
        "//software/geom",
    ],
)

cc_test(
    name = "chip_primitive_test",
    srcs = ["chip_primitive_test.cpp"],
    deps = [
        ":chip_primitive",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "direct_velocity_primitive",
    srcs = ["direct_velocity_primitive.cpp"],
    hdrs = ["direct_velocity_primitive.h"],
    deps = [
        ":primitive",
        "//software/geom",
    ],
)

cc_test(
    name = "direct_velocity_primitive_test",
    srcs = ["direct_velocity_primitive_test.cpp"],
    deps = [
        ":direct_velocity_primitive",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "direct_wheels_primitive",
    srcs = ["direct_wheels_primitive.cpp"],
    hdrs = ["direct_wheels_primitive.h"],
    deps = [
        ":primitive",
        "//software/geom",
    ],
)

cc_test(
    name = "direct_wheels_primitive_test",
    srcs = ["direct_wheels_primitive_test.cpp"],
    deps = [
        ":direct_wheels_primitive",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "dribble_primitive",
    srcs = ["dribble_primitive.cpp"],
    hdrs = ["dribble_primitive.h"],
    deps = [
        ":primitive",
        "//software/geom",
    ],
)

cc_test(
    name = "dribble_primitive_test",
    srcs = ["dribble_primitive_test.cpp"],
    deps = [
        ":dribble_primitive",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "kick_primitive",
    srcs = ["kick_primitive.cpp"],
    hdrs = ["kick_primitive.h"],
    deps = [
        ":primitive",
        "//software/geom",
    ],
)

cc_test(
    name = "kick_primitive_test",
    srcs = ["kick_primitive_test.cpp"],
    deps = [
        ":kick_primitive",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "move_primitive",
    srcs = ["move_primitive.cpp"],
    hdrs = ["move_primitive.h"],
    deps = [
        ":primitive",
        "//software/geom",
    ],
)

cc_test(
    name = "move_primitive_test",
    srcs = ["move_primitive_test.cpp"],
    deps = [
        ":move_primitive",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "movespin_primitive",
    srcs = ["movespin_primitive.cpp"],
    hdrs = ["movespin_primitive.h"],
    deps = [
        ":primitive",
        "//software/geom",
        "//software/new_geom:angular_velocity",
    ],
)

cc_test(
    name = "movespin_primitive_test",
    srcs = ["movespin_primitive_test.cpp"],
    deps = [
        ":movespin_primitive",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "pivot_primitive",
    srcs = ["pivot_primitive.cpp"],
    hdrs = ["pivot_primitive.h"],
    deps = [
        ":primitive",
        "//shared:constants",
        "//software/geom",
    ],
)

cc_test(
    name = "pivot_primitive_test",
    srcs = ["pivot_primitive_test.cpp"],
    deps = [
        ":pivot_primitive",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "stop_primitive",
    srcs = ["stop_primitive.cpp"],
    hdrs = ["stop_primitive.h"],
    deps = [
        ":primitive",
        "//software/geom",
    ],
)

cc_test(
    name = "stop_primitive_test",
    srcs = ["stop_primitive_test.cpp"],
    deps = [
        ":stop_primitive",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "all_primitives",
    hdrs = ["all_primitives.h"],
    deps = [
        ":catch_primitive",
        ":chip_primitive",
        ":direct_velocity_primitive",
        ":direct_wheels_primitive",
        ":dribble_primitive",
        ":kick_primitive",
        ":move_primitive",
        ":movespin_primitive",
        ":pivot_primitive",
        ":stop_primitive",
    ],
)

cc_library(
    name = "primitive_visitor",
    hdrs = ["primitive_visitor.h"],
)

cc_library(
    name = "primitive",
    hdrs = ["primitive.h"],
    deps = ["primitive_visitor"],
)
