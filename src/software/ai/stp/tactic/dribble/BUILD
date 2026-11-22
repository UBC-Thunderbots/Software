load("@gameplay_tests_deps//:requirements.bzl", "requirement")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")

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
        "//software/ai/stp/tactic",
        "//software/ai/stp/tactic/move:move_tactic",
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
    name = "dribble_tactic_test",
    srcs = [
        "dribble_tactic_test.py",
    ],
    main = "dribble_tactic_test.py",
    args = ["--test_filter='test_dribble and not excessive'"],
    tags = ["exclusive"],
    deps = [
        "//software:conftest",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)

py_test(
    name = "dribble_excessive_dribbling_test",
    srcs = [
        "dribble_tactic_test.py",
    ],
    main = "dribble_tactic_test.py",
    args = ["--test_filter=test_excessive_dribbling_without_enemies"],
    tags = ["exclusive"],
    deps = [
        "//software:conftest",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)

# TODO: #4081 - resolve thread limit issue and recombine dribble tests
py_test(
    name = "dribble_with_excessive_dribbling_test",
    srcs = [
        "dribble_tactic_test.py",
    ],
    main = "dribble_tactic_test.py",
    args = ["--test_filter=test_dribble_with_excessive_dribbling"],
    tags = ["exclusive"],
    deps = [
        "//software:conftest",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)

py_test(
    name = "dribble_knock_ball_away_test",
    srcs = [
        "dribble_tactic_test.py",
    ],
    main = "dribble_tactic_test.py",
    args = ["--test_filter=test_run_into_enemy_robot_knock_ball_away"],
    tags = ["exclusive"],
    deps = [
        "//software:conftest",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)

py_test(
    name = "dribble_turning_bump_test",
    srcs = [
        "dribble_tactic_test.py",
    ],
    main = "dribble_tactic_test.py",
    args = ["--test_filter=test_robot_not_bumping_ball_when_turning"],
    tags = ["exclusive"],
    deps = [
        "//software:conftest",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)
