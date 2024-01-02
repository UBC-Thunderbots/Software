package(default_visibility = ["//visibility:public"])

cc_library(
    name = "skill",
    srcs = ["skill.cpp"],
    hdrs = ["skill.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//software/ai/hl/stp/strategy:strategy",
        "//software/ai/hl/stp/tactic:tactic_fsm",
        "//software/ai/hl/stp/tactic/dribble:dribble_fsm",
        "//software/util/generic_factory",
    ],
)

cc_test(
    name = "skill_test",
    srcs = ["skill_test.cpp"],
    deps = [
        ":skill",
        "//shared/test_util:tbots_gtest_main",
    ]
)

cc_library(
    name = "dribble_skill",
    srcs = ["dribble_skill.cpp"],
    hdrs = ["dribble_skill.h"],
    deps = [":skill"],
)

cc_library(
    name = "pass_skill",
    hdrs = ["pass_skill.h"],
    deps = [":skill"],
)

cc_library(
    name = "winger_skill",
    hdrs = ["winger_skill.h"],
    deps = [":skill"],
)

cc_library(
    name = "shoot_skill",
    hdrs = ["shoot_skill.h"],
    deps = [":skill"],
)
