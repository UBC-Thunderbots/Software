load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dribble_tactic",
    srcs = [
        "dribble_fsm.cpp",
        "dribble_tactic.cpp",
    ],
    hdrs = [
        "dribble_fsm.h",
        "dribble_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:time_to_travel",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
    ],
)

cc_test(
    name = "dribble_fsm_test",
    srcs = ["dribble_fsm_test.cpp"],
    deps = [
        ":dribble_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

py_test(
    name = "excessive_dribble_test",
    srcs = [
        "excessive_dribble_test.py",
    ],
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
