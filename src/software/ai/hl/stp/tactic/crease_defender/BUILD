package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "crease_defender_tactic",
    srcs = [
        "crease_defender_fsm.cpp",
        "crease_defender_tactic.cpp",
    ],
    hdrs = [
        "crease_defender_fsm.h",
        "crease_defender_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/geom:stadium",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/ai/hl/stp/tactic/defender:defender_base",
    ],
)

cc_test(
    name = "crease_defender_fsm_test",
    srcs = ["crease_defender_fsm_test.cpp"],
    deps = [
        ":crease_defender_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "crease_defender_tactic_cpp_test",
    srcs = ["crease_defender_tactic_test.cpp"],
    deps = [
        ":crease_defender_tactic",
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

py_test(
    name = "crease_defender_tactic_test",
    srcs = ["crease_defender_tactic_test.py"],
    tags = [
        "exclusive",
    ],
    deps = [
        "//software/simulated_tests:simulated_test_fixture",
        "//software/simulated_tests:speed_threshold_helpers",
        "//software/simulated_tests:validation",
        requirement("pytest"),
    ],
)
