package(default_visibility = ["//visibility:public"])

cc_library(
    name = "move_rule",
    srcs = ["move_rule.cpp"],
    hdrs = ["move_rule.h"],
    deps = [],
)

cc_library(
    name = "move_rule_manager",
    srcs = ["move_rule_manager.cpp"],
    hdrs = ["move_rule_manager.h"],
    deps = [
        ":move_rule",
        "//software/ai/hl/stp/tactic:all_tactics",
        "//software/ai/hl/stp/tactic:tactic_visitor",
        "//software/ai/world:game_state",
    ],
)
