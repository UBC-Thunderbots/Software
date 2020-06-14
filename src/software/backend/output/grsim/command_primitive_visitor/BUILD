package(default_visibility = ["//visibility:public"])

cc_library(
    name = "grsim_command_primitive_visitor",
    srcs = ["grsim_command_primitive_visitor.cpp"],
    hdrs = ["grsim_command_primitive_visitor.h"],
    deps = [
        ":motion_controller",
        "//shared:constants",
        "//software/new_geom:angle",
        "//software/new_geom:point",
        "//software/new_geom/util:closest_point",
        "//software/primitive:all_primitives",
        "//software/primitive:primitive_visitor",
        "//software/world:ball",
        "//software/world:robot",
    ],
)

cc_library(
    name = "motion_controller",
    srcs = ["motion_controller.cpp"],
    hdrs = ["motion_controller.h"],
    deps = [
        "//software/util/variant_visitor",
        "//software/world:robot",
    ],
)

cc_test(
    name = "catch_primitive_test",
    srcs = ["catch_primitive_test.cpp"],
    deps = [
        ":grsim_command_primitive_visitor",
        "//software/primitive:catch_primitive",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "dribble_primitive_test",
    srcs = ["dribble_primitive_test.cpp"],
    deps = [
        ":grsim_command_primitive_visitor",
        "//software/primitive:dribble_primitive",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "motion_controller_test",
    srcs = ["motion_controller_test.cpp"],
    deps = [
        ":motion_controller",
        "//software/primitive:move_primitive",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "movespin_primitive_test",
    srcs = ["movespin_primitive_test.cpp"],
    deps = [
        ":grsim_command_primitive_visitor",
        "//software/primitive:movespin_primitive",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "pivot_primitive_test",
    srcs = ["pivot_primitive_test.cpp"],
    deps = [
        ":grsim_command_primitive_visitor",
        "//software/primitive:pivot_primitive",
        "@gtest//:gtest_main",
    ],
)
