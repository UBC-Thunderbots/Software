load("@gameplay_tests_deps//:requirements.bzl", "requirement")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "example_play",
    srcs = [
        "example_play.cpp",
        "example_play_fsm.cpp",
    ],
    hdrs = [
        "example_play.h",
        "example_play_fsm.h",
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
    name = "example_play_fsm_test",
    srcs = ["example_play_fsm_test.cpp"],
    deps = [
        ":example_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

py_test(
    name = "example_play_test",
    srcs = ["example_play_test.py"],
    deps = [
        "//software:conftest",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)
