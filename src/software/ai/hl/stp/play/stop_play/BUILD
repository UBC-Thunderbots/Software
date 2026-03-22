load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stop_play",
    srcs = [
        "stop_play.cpp",
        "stop_play_fsm.cpp",
    ],
    hdrs = [
        "stop_play.h",
        "stop_play_fsm.h",
    ],
    deps = [
        "//software/ai/hl/stp/play",
        "//shared:constants",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "stop_play_cpp_test",
    srcs = ["stop_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play/stop_play:stop_play",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/cpp_validation:validation_function",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

py_test(
    name = "stop_play_test",
    srcs = [
        "stop_play_test.py",
    ],
    # The default main would be stop_play_test.py; override to our file.
    main = "stop_play_test.py",
    # TODO (#2619) Remove tag to run in parallel
    tags = [
        "exclusive",
    ],
    deps = [
        "//software:conftest",
        "//software/simulated_tests/validation:validations",
        requirement("pytest"),
    ],
)

