package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pass_strategy",
    srcs = ["pass_strategy.cpp"],
    hdrs = ["pass_strategy.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//software/ai/passing:eighteen_zone_pitch_division",
        "//software/ai/passing:pass",
        "//software/ai/passing:pass_evaluation",
        "//software/ai/passing:pass_generator",
        "//software/world:world"
    ]
)

cc_library(
    name = "strategy",
    srcs = ["strategy.cpp"],
    hdrs = ["strategy.h"],
    deps = [
        ":strategy_impl",
        "//proto:tbots_cc_proto",
        "//software/ai/passing:eighteen_zone_pitch_division",
        "//software/ai/passing:pass",
        "//software/ai/passing:pass_generator",
        "//software/world:robot",
        "//software/world:robot_state",
    ],
)

cc_library(
    name = "strategy_impl",
    srcs = ["strategy_impl.cpp"],
    hdrs = ["strategy_impl.h"],
    visibility = ["//visibility:private"],
    deps = [
        ":pass_strategy",
        "//proto:tbots_cc_proto",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/evaluation:shot",
        "//software/ai/hl/stp/tactic:offense_support_tactic",
        "//software/ai/passing:pass_with_rating",
        "//software/geom:pose",
        "//software/world:field",
    ]
)
