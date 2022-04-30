package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stop_tactic",
    srcs = [
        "stop_fsm.cpp",
        "stop_tactic.cpp",
    ],
    hdrs = [
        "stop_fsm.h",
        "stop_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/tactic",
        "//software/logger",
    ],
)

cc_test(
    name = "stop_fsm_test",
    srcs = ["stop_fsm_test.cpp"],
    deps = [
        ":stop_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "stop_tactic_test",
    srcs = ["stop_tactic_test.cpp"],
    deps = [
        ":stop_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
    ],
)
