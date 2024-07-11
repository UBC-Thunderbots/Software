package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "defender_base",
    srcs = [
        "defender_base.cpp",
    ],
    hdrs = [
        "defender_base.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic:move_primitive",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

