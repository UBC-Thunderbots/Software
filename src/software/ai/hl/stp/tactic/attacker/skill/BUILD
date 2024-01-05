package(default_visibility = ["//visibility:public"])

cc_library(
    name = "all_skills",
    deps = [
        ":dribble_skill",
        ":pass_skill",
        ":shoot_skill",
        ":winger_skill"
    ]
)

cc_library(
    name = "head_skill",
    srcs = ["head_skill.cpp"],
    hdrs = ["head_skill.h"],
    deps = [":skill"],
)

cc_library(
    name = "skill",
    srcs = ["skill.cpp"],
    hdrs = ["skill.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//software/ai/hl/stp/strategy",
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
    ],
)

cc_library(
    name = "dribble_skill",
    srcs = ["dribble_skill.cpp"],
    hdrs = ["dribble_skill.h"],
    deps = [":skill"],
    alwayslink = True,
)

cc_test(
    name = "dribble_skill_test",
    srcs = ["dribble_skill_test.cpp"],
    deps = [
        ":dribble_skill",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "pass_skill",
    hdrs = ["pass_skill.h"],
    deps = [":skill"],
    alwayslink = True,
)

cc_test(
    name = "pass_skill_test",
    srcs = ["pass_skill_test.cpp"],
    deps = [
        ":pass_skill",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "winger_skill",
    srcs = ["winger_skill.cpp"],
    hdrs = ["winger_skill.h"],
    deps = [":skill"],
    alwayslink = True,
)

cc_test(
    name = "winger_skill_test",
    srcs = ["winger_skill_test.cpp"],
    deps = [
        ":winger_skill",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "shoot_skill",
    hdrs = ["shoot_skill.h"],
    deps = [":skill"],
    alwayslink = True,
)

cc_test(
    name = "shoot_skill_test",
    srcs = ["shoot_skill_test.cpp"],
    deps = [
        ":shoot_skill",
        "//shared/test_util:tbots_gtest_main",
    ],
)
