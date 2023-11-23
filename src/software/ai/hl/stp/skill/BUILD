package(default_visibility = ["//visibility:public"])

cc_library(
    name = "all_skills",
    hdrs = ["all_skills.h"],
    deps = [
        ":dribble_skill"
    ]
)

cc_library(
    name = "dribble_skill",
    hdrs = ["dribble_skill.h"],
)

cc_library(
    name = "skill_visitor",
    hdrs = ["skill_visitor.h"],
    srcs = ["skill_visitor.cpp"],
    deps = [
        ":all_skills"
    ],
)
