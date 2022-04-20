package(default_visibility = ["//visibility:public"])

cc_library(
    name = "motion_constraint_visitor",
    srcs = ["motion_constraint_visitor.cpp"],
    hdrs = ["motion_constraint_visitor.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//software/ai/hl/stp/tactic:all_tactics",
        "//software/world:game_state",
    ],
)

cc_library(
    name = "motion_constraint_set_builder",
    srcs = ["motion_constraint_set_builder.cpp"],
    hdrs = ["motion_constraint_set_builder.h"],
    deps = [
        ":motion_constraint_visitor",
    ],
)

cc_test(
    name = "motion_constraint_set_builder_test",
    srcs = ["motion_constraint_set_builder_test.cpp"],
    deps = [
        ":motion_constraint_set_builder",
        ":motion_constraint_visitor",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
