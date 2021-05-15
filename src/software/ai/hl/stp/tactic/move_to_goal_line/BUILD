package(default_visibility = ["//visibility:public"])

cc_library(
    name = "move_to_goal_line_tactic",
    srcs = ["move_to_goal_line_tactic.cpp"],
    hdrs = [
        "move_to_goal_line_fsm.h",
        "move_to_goal_line_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/geom:line",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "move_to_goal_line_fms_test",
    srcs = ["move_to_goal_line_fsm_test.cpp"],
    deps = [
        ":move_to_goal_line_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "simulated_move_to_goal_line_tactic_test",
    srcs = ["simulated_move_to_goal_line_tactic_test.cpp"],
    deps = [
        ":move_to_goal_line_tactic",
        "//software/simulated_tests:simulated_tactic_test_fixture",
        "//software/simulated_tests:simulated_test_main",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
