package(default_visibility = ["//visibility:public"])

cc_library(
    name = "offense_support_type",
    hdrs = ["offense_support_type.h"],
    deps = ["//software/util/make_enum"],
)

cc_library(
    name = "offense_support_tactic",
    srcs = ["offense_support_tactic.cpp"],
    hdrs = ["offense_support_tactic.h"],
    deps = [
        ":offense_support_type",
        "//software/ai/hl/stp/strategy",
        "//software/ai/hl/stp/tactic",
    ],
)
