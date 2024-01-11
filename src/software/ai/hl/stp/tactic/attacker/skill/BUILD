package(default_visibility = ["//visibility:public"])

cc_library(
    name = "head_skill",
    srcs = ["head_skill.cpp"],
    hdrs = ["head_skill.h"],
    deps = [":skill"],
)

cc_library(
    name = "skill",
    srcs = [
        "dribble_skill.cpp",
        "pass_skill.h",
        "shoot_skill.cpp",
        "skill.cpp",
        "winger_skill.cpp",
    ],
    hdrs = [
        "dribble_skill.h",
        "pass_skill.h",
        "skill.h",
        "shoot_skill.h",
        "winger_skill.h"
    ],
    deps = [
        "//proto:tbots_cc_proto",
        "//software/ai/hl/stp/strategy",
        "//software/ai/hl/stp/tactic:tactic_fsm",
        "//software/ai/hl/stp/tactic/dribble:dribble_fsm",
        "//software/ai/hl/stp/tactic/attacker:attacker_fsm",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "skill_test",
    srcs = ["skill_test.cpp"],
    deps = [
        ":skill",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "dribble_skill_test",
    srcs = ["dribble_skill_test.cpp"],
    deps = [
        ":skill",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "pass_skill_test",
    srcs = ["pass_skill_test.cpp"],
    deps = [
        ":skill",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "winger_skill_test",
    srcs = ["winger_skill_test.cpp"],
    deps = [
        ":skill",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "shoot_skill_test",
    srcs = ["shoot_skill_test.cpp"],
    deps = [
        ":skill",
        "//shared/test_util:tbots_gtest_main",
    ],
)
