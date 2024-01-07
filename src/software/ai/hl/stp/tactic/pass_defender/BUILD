package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "pass_defender_tactic",
    srcs = [
        "pass_defender_fsm.cpp",
        "pass_defender_tactic.cpp",
    ],
    hdrs = [
        "pass_defender_fsm.h",
        "pass_defender_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "pass_defender_fsm_test",
    srcs = ["pass_defender_fsm_test.cpp"],
    deps = [
        ":pass_defender_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

py_test(
    name = "pass_defender_tactic_test",
    srcs = [
        "pass_defender_tactic_test.py",
    ],
    # TODO (#2619) Remove tag to run in parallel
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
