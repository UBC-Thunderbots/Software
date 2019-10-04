package(default_visibility = ["//visibility:public"])

cc_library(
    name = "move_test_tactic",
    srcs = ["move_test_tactic.cpp"],
    hdrs = ["move_test_tactic.h"],
    deps = [
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic:tactic_visitor",
        "//software/ai/intent:move_intent",
    ],
)

cc_library(
    name = "stop_test_tactic",
    srcs = ["stop_test_tactic.cpp"],
    hdrs = ["stop_test_tactic.h"],
    deps = [
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic:tactic_visitor",
        "//software/ai/intent:stop_intent",
    ],
)
