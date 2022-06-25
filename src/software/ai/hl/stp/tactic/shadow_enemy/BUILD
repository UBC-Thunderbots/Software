package(default_visibility = ["//visibility:public"])

cc_library(
    name = "shadow_enemy_tactic",
    srcs = [
        "shadow_enemy_fsm.cpp",
        "shadow_enemy_tactic.cpp",
    ],
    hdrs = [
        "shadow_enemy_fsm.h",
        "shadow_enemy_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
    ],
)

cc_test(
    name = "shadow_enemy_fsm_test",
    srcs = ["shadow_enemy_fsm_test.cpp"],
    deps = [
        ":shadow_enemy_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "shadow_enemy_tactic_test",
    srcs = ["shadow_enemy_tactic_test.cpp"],
    deps = [
        ":shadow_enemy_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/geom:triangle",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
