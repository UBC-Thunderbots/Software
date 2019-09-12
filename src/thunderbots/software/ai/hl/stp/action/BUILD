package(default_visibility = ["//visibility:public"])

cc_library(
    name = "chip_action",
    srcs = ["chip_action.cpp"],
    hdrs = ["chip_action.h"],
    deps = [
        ":action",
        "//shared:constants",
        "//software/ai/intent:chip_intent",
        "//software/ai/intent:move_intent",
    ],
)

cc_library(
    name = "dribble_action",
    srcs = ["dribble_action.cpp"],
    hdrs = ["dribble_action.h"],
    deps = [
        ":action",
        "//software/ai/intent:dribble_intent",
    ],
)

cc_library(
    name = "kick_action",
    srcs = ["kick_action.cpp"],
    hdrs = ["kick_action.h"],
    deps = [
        ":action",
        "//shared:constants",
        "//software/ai/intent:kick_intent",
        "//software/ai/intent:move_intent",
        "//software/ai/world:ball",
    ],
)

cc_library(
    name = "move_action",
    srcs = ["move_action.cpp"],
    hdrs = ["move_action.h"],
    deps = [
        ":action",
        "//software/ai/intent:move_intent",
    ],
)

cc_library(
    name = "movespin_action",
    srcs = ["movespin_action.cpp"],
    hdrs = ["movespin_action.h"],
    deps = [
        ":action",
        "//software/ai/intent:movespin_intent",
    ],
)

cc_library(
    name = "pivot_action",
    srcs = ["pivot_action.cpp"],
    hdrs = ["pivot_action.h"],
    deps = [
        ":action",
        "//shared:constants",
        "//software/ai/intent:move_intent",
        "//software/ai/intent:pivot_intent",
        "//software/util/parameter:dynamic_parameters",
        "@g3log//:all",
    ],
)

cc_library(
    name = "stop_action",
    srcs = ["stop_action.cpp"],
    hdrs = ["stop_action.h"],
    deps = [
        ":action",
        "//software/ai/intent:stop_intent",
    ],
)

cc_library(
    name = "action",
    srcs = ["action.cpp"],
    hdrs = ["action.h"],
    deps = [
        "//software/ai/intent",
        "//software/ai/world:ball",
        "//software/ai/world:robot",
    ],
)
