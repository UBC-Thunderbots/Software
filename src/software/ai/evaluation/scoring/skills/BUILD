package(default_visibility = ["//visibility:public"])

cc_library(
    name = "feasibility_visitor",
    srcs = ["feasibility_visitor.cpp"],
    hdrs = ["feasibility_visitor.h"],
    deps = [
        ":skill_visitor",
        "//software/ai:strategy",
        "//software/ai/evaluation:keep_away",
        "//software/ai/hl/stp/skill",
        "//software/world",
        "//software/world:robot",
    ],
)

cc_library(
    name = "skill_visitor",
    hdrs = ["skill_visitor.h"],
)

cc_library(
    name = "skill_graph",
    srcs = ["skill_graph.cpp"],
    hdrs = ["skill_graph.h"],
    deps = [
        ":feasibility_visitor",
        "//software/ai/hl/stp/skill",
        "//software/ai/hl/stp/skill:all_skills",
        "//software/util/generic_factory",
    ],
)
