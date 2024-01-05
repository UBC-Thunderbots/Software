package(default_visibility = ["//visibility:public"])

cc_library(
    name = "strategy",
    srcs = ["strategy.cpp"],
    hdrs = ["strategy.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/hl/stp/tactic:offense_support_tactic",
        "//software/ai/passing:eighteen_zone_pitch_division",
        "//software/ai/passing:pass",
        "//software/ai/passing:pass_generator",
        "//software/geom:pose",
        "//software/world:robot",
        "//software/world:robot_state",
    ],
)
