package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "defender_fsm_base",
    srcs = [
        "defender_fsm_base.cpp",
    ],
    hdrs = [
        "defender_fsm_base.h",
    ],
    deps = [
        "//software/ai/hl/stp/skill/dribble:dribble_skill",
        "//software/ai/hl/stp/tactic",
    ],
)
