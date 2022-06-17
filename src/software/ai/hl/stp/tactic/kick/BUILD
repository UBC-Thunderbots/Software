package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "kick_tactic",
    srcs = [
        "kick_fsm.cpp",
        "kick_tactic.cpp",
    ],
    hdrs = [
        "kick_fsm.h",
        "kick_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/get_behind_ball:get_behind_ball_tactic",
        "//software/logger",
    ],
)

cc_test(
    name = "kick_fsm_test",
    srcs = ["kick_fsm_test.cpp"],
    deps = [
        ":kick_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

#cc_test(
#    name = "kick_tactic_test",
#    srcs = ["kick_tactic_test.cpp"],
#    deps = [
#        ":kick_tactic",
#        "//shared/test_util:tbots_gtest_main",
#        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
#        "//software/simulated_tests/terminating_validation_functions",
#        "//software/simulated_tests/validation:validation_function",
#        "//software/test_util",
#    ],
#)

py_test(
    name = "kick_tactic_test",
    srcs = [
        "kick_tactic_test.py",
    ],
    # TODO (#2619) Remove tag to run in parallel
    tags = [
        "exclusive",
    ],
    deps = [
        "//software/simulated_tests:simulated_test_fixture",
        "//software/simulated_tests:validation",
        "//software/simulated_tests/field_tests:field_test_fixture",
        requirement("pytest"),
    ],
)

py_test(
    name = "kick_tactic_test_field",
    srcs = [
        "kick_tactic_test_field.py",
    ],
    # TODO (#2619) Remove tag to run in parallel
    tags = [
        "exclusive",
    ],
    deps = [
        "//software/simulated_tests:simulated_test_fixture",
        "//software/simulated_tests:validation",
        "//software/simulated_tests/field_tests:field_test_fixture",
        requirement("pytest"),
    ],
)
