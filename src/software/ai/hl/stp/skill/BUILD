package(default_visibility = ["//visibility:public"])

cc_library(
    name = "skill",
    hdrs = ["skill.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//software/ai/hl/stp/tactic/dribble:dribble_fsm",
        "//software/ai/hl/stp/tactic:tactic_fsm",
        "//software/ai/hl/stp:strategy",
        "//software/util/generic_factory",
    ]
)

cc_library(
    name = "dribble_skill",
    hdrs = ["dribble_skill.h"],
    srcs = ["dribble_skill.cpp"],
    deps = [":skill"]
)

cc_library(
    name = "pass_skill",
    hdrs = ["pass_skill.h"],
    deps = [":skill"]
)

cc_library(
    name = "winger_skill",
    hdrs = ["winger_skill.h"],
    deps = [":skill"]
)

cc_library(
    name = "shoot_skill",
    hdrs = ["shoot_skill.h"],
    deps = [":skill"]
)
