package(default_visibility = ["//visibility:public"])

cc_library(
    name = "motion_constraint",
    hdrs = ["motion_constraint.h"],
    deps = ["//software/util/make_enum"],
)

cc_library(
    name = "motion_constraint_manager",
    srcs = ["motion_constraint_manager.cpp"],
    hdrs = ["motion_constraint_manager.h"],
    deps = [
        ":motion_constraint",
        "//software/ai/hl/stp/tactic:all_tactics",
        "//software/world:game_state",
    ],
)

cc_test(
    name = "motion_constraint_manager_test",
    srcs = ["motion_constraint_manager_test.cpp"],
    deps = [
        ":motion_constraint_manager",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
