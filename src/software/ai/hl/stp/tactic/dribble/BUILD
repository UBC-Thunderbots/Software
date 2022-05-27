package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dribble_tactic",
    srcs = [
        "dribble_fsm.cpp",
        "dribble_tactic.cpp",
    ],
    hdrs = [
        "dribble_fsm.h",
        "dribble_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:time_to_travel",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
    ],
)

cc_test(
    name = "dribble_fsm_test",
    srcs = ["dribble_fsm_test.cpp"],
    deps = [
        ":dribble_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "dribble_tactic_test",
    srcs = ["dribble_tactic_test.cpp"],
    deps = [
        ":dribble_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "dribble_tactic_push_enemy_test",
    srcs = ["dribble_tactic_push_enemy_test.cpp"],
    deps = [
        ":dribble_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
