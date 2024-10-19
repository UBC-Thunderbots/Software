package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "halt_play",
    srcs = ["halt_play.cpp"],
    hdrs = ["halt_play.h"],
    deps = [
        "//software/ai/hl/stp/play:play",
        "//shared:constants",
        "//software/ai/hl/stp/tactic/stop:stop_tactic",
        "//software/logger",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "halt_play_cpp_test",
    srcs = ["halt_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        ":halt_play",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
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