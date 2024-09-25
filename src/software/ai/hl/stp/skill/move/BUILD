package(default_visibility = ["//visibility:public"])

cc_library(
    name = "move_skill",
    srcs = [
        "move_skill_fsm.cpp",
    ],
    hdrs = [
        "move_skill.h",
        "move_skill_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/primitive:move_primitive",
        "//software/ai/hl/stp/skill",
        "//software/ai/hl/stp/tactic:transition_conditions",
        "//software/logger",
    ],
)

cc_test(
    name = "move_skill_fsm_test",
    srcs = ["move_skill_fsm_test.cpp"],
    deps = [
        ":move_skill",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "move_skill_test",
    srcs = ["move_skill_test.cpp"],
    deps = [
        ":move_skill",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/tactic/assigned_skill:assigned_skill_tactic",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
    ],
)
