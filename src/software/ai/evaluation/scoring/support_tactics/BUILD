package(default_visibility = ["//visibility:public"])

cc_library(
    name = "support_tactic_candidate",
    hdrs = ["support_tactic_candidate.hpp"],
    deps = [
        ":support_tactic_scorer",
        "//software/ai/evaluation/scoring:candidate",
        "//software/ai/hl/stp/tactic/receiver:receiver_tactic",
    ],
)

cc_library(
    name = "support_tactic_scorer",
    hdrs = ["support_tactic_scorer.h"],
    deps = [
        "//software/ai/hl/stp/tactic/receiver:receiver_tactic",
    ],
)

cc_library(
    name = "duplication_scorer",
    srcs = ["duplication_scorer.cpp"],
    hdrs = ["duplication_scorer.h"],
    deps = [
        ":support_tactic_candidate",
        ":support_tactic_scorer",
        "//software/util/type_map",
    ],
)

cc_library(
    name = "feasibility_scorer",
    srcs = ["feasibility_scorer.cpp"],
    hdrs = ["feasibility_scorer.h"],
    deps = [
        ":support_tactic_candidate",
        ":support_tactic_scorer",
    ],
)

cc_library(
    name = "success_scorer",
    srcs = ["success_scorer.cpp"],
    hdrs = ["success_scorer.h"],
    deps = [
        ":support_tactic_candidate",
        ":support_tactic_scorer",
        "//software/util/type_map",
    ],
)
