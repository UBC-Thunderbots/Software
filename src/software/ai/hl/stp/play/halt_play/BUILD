package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "halt_play",
    srcs = [
        "halt_play.cpp",
        "halt_play_fsm.cpp",
    ],
    hdrs = [
        "halt_play.h",
        "halt_play_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/tactic/halt:halt_tactic",
        "//software/logger",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "halt_play_cpp_test",
    srcs = ["halt_play_test.cpp"],
    deps = [
        ":halt_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "halt_play_fsm_test",
    srcs = ["halt_play_fsm_test.cpp"],
    deps = [
        ":halt_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

py_test(
    name = "halt_play_test",
    srcs = [
        "halt_play_test.py",
    ],
    # TODO (#2619) Remove tag to run in parallel
    tags = [
        "exclusive",
    ],
    deps = [
        "//software:conftest",
        "//software/simulated_tests:speed_threshold_helpers",
        "//software/simulated_tests:validation",
        requirement("pytest"),
    ],
)
