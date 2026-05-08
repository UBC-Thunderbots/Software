load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "defender_fsm_base",
    srcs = [
        "defender_fsm_base.cpp",
    ],
    hdrs = [
        "defender_fsm_base.h",
    ],
    deps = [
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
    ],
)
