load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "halt_tactic",
    srcs = [
        "halt_fsm.cpp",
        "halt_tactic.cpp",
    ],
    hdrs = [
        "halt_fsm.h",
        "halt_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/tactic",
        "//software/logger",
    ],
)

cc_test(
    name = "halt_fsm_test",
    srcs = ["halt_fsm_test.cpp"],
    deps = [
        ":halt_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

py_test(
    name = "halt_tactic_test",
    srcs = ["halt_tactic_test.py"],
    deps = [
        "//software:conftest",
        "//software/simulated_tests/validation:validations",
        requirement("pytest"),
    ],
)
