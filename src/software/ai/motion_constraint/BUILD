package(default_visibility = ["//visibility:public"])

cc_library(
    name = "motion_constraint",
    srcs = ["motion_constraint.cpp"],
    hdrs = ["motion_constraint.h"],
    deps = ["@magic_enum"],
)

cc_library(
    name = "motion_constraint_manager",
    srcs = ["motion_constraint_manager.cpp"],
    hdrs = ["motion_constraint_manager.h"],
    deps = [
        ":motion_constraint",
        "//software/ai/hl/stp/tactic:all_tactics",
        "//software/ai/hl/stp/tactic:tactic_visitor",
        "//software/world:game_state",
    ],
)

cc_test(
    name = "motion_constraint_manager_test",
    srcs = ["motion_constraint_manager_test.cpp"],
    deps = [
        ":motion_constraint_manager",
        "@gtest//:gtest_main",
        "//software/test_util",
    ],
)
