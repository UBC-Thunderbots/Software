package(default_visibility = ["//visibility:public"])

# We force linking for all Skills so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "pivot_kick_skill",
    srcs = [
        "pivot_kick_skill_fsm.cpp",
    ],
    hdrs = [
        "pivot_kick_skill_fsm.h",
    ],
    deps = [
        "//software/ai/hl/stp/primitive:move_primitive",
        "//software/ai/hl/stp/skill",
        "//software/ai/hl/stp/skill/dribble:dribble_skill",
    ],
    alwayslink = True,
)

cc_test(
    name = "pivot_kick_skill_fsm_test",
    srcs = ["pivot_kick_skill_fsm_test.cpp"],
    deps = [
        ":pivot_kick_skill",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "pivot_kick_skill_test",
    srcs = ["pivot_kick_skill_test.cpp"],
    deps = [
        ":pivot_kick_skill",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/tactic/assigned_skill:assigned_skill_tactic",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
