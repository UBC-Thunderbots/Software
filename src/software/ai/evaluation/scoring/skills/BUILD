package(default_visibility = ["//visibility:public"])

cc_library(
    name = "feasibility_visitor",
    hdrs = ["feasibility_visitor.h"],
    srcs = ["feasibility_visitor.cpp"],
    deps = [
        ":skill_visitor",
        "//software/ai/hl/stp/skill:skill",
        "//software/ai/hl/stp/strategy:strategy",
        "//software/world:robot",
        "//software/world:world"
    ]
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
