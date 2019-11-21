package(default_visibility = ["//software/ai/hl/stp/action:__subpackages__"])

cc_library(
    name = "move_test_action",
    testonly = 1,
    srcs = ["move_test_action.cpp"],
    hdrs = ["move_test_action.h"],
    deps = [
        "//software/ai/hl/stp/action",
        "//software/ai/hl/stp/action:action_visitor",
        "//software/ai/intent:move_intent",
        "//software/geom",
    ],
)
