package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "example_play",
    srcs = [
        "example_play.cpp",
<<<<<<< Updated upstream
        "example_play_fsm.cpp",
    ],
    hdrs = [
        "example_play.h",
        "example_play_fsm.h",
=======
        "example_play_fsm.cpp"
    ],
    hdrs = [
        "example_play.h",
        "example_play_fsm.h"
>>>>>>> Stashed changes
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "example_play_test",
    srcs = ["example_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play/example:example_play",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "example_play_fsm_test",
    srcs = ["example_play_fsm_test.cpp"],
    deps = [
        ":example_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

py_test(
    name = "example_play_test_py",
    main = "example_play_test.py",
    srcs = ["example_play_test.py"],
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