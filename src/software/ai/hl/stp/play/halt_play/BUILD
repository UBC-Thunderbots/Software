load("@gameplay_tests_deps//:requirements.bzl", "requirement")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")

package(default_visibility = ["//visibility:public"])

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
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/halt:halt_tactic",
        "//software/logger",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
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
    deps = [
        "//software:conftest",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)
