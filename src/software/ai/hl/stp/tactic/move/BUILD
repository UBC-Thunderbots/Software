load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "move_tactic",
    srcs = [
        "move_fsm.cpp",
        "move_tactic.cpp",
    ],
    hdrs = [
        "move_fsm.h",
        "move_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic:move_primitive",
        "//software/logger",
    ],
)

cc_test(
    name = "move_fsm_test",
    srcs = ["move_fsm_test.cpp"],
    deps = [
        ":move_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
