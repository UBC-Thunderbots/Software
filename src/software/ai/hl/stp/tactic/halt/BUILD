package(default_visibility = ["//visibility:public"])

cc_library(
    name = "halt_tactic",
    srcs = [
        "halt_fsm.cpp",
        "halt_tactic.cpp",
        "stop_fsm.cpp",
    ],
    hdrs = [
        "halt_fsm.h",
        "halt_tactic.h",
        "stop_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/tactic",
        "//software/logger",
    ],
)

cc_test(
    name = "halt_fsm_test",
    srcs = ["halt_fsm_test.cpp"],
    deps = [
        ":halt_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "halt_tactic_test",
    srcs = ["halt_tactic_test.cpp"],
    deps = [
        ":halt_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
    ],
)
