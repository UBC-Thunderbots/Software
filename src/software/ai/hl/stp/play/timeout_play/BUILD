package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "timeout_play",
    srcs = [
        "timeout_play.cpp",
        "timeout_play_fsm.cpp",
    ],
    hdrs = [
        "timeout_play.h",
        "timeout_play_fsm.h",
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
