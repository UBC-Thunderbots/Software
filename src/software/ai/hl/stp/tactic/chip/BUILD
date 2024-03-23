package(default_visibility = ["//visibility:public"])

cc_library(
    name = "chip_tactic",
    srcs = [
        "chip_fsm.cpp",
        "chip_tactic.cpp",
    ],
    hdrs = [
        "chip_fsm.h",
        "chip_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/primitive:move_primitive",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/get_behind_ball:get_behind_ball_tactic",
        "//software/logger",
    ],
)

cc_test(
    name = "chip_fsm_test",
    srcs = ["chip_fsm_test.cpp"],
    deps = [
        ":chip_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "chip_tactic_test",
    srcs = ["chip_tactic_test.cpp"],
    deps = [
        ":chip_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
    ],
)
