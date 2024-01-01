package(default_visibility = ["//visibility:public"])

cc_library(
    name = "skill",
    hdrs = ["skill.h"],
    deps = ["//software/ai/hl/stp:robot_control_strategy"]
)

cc_library(
    name = "dribble_skill",
    hdrs = ["dribble_skill.h"],
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
