package(default_visibility = ["//visibility:public"])

cc_library(
    name = "avoid_area",
    srcs = ["avoid_area.cpp"],
    hdrs = ["avoid_area.h"],
    deps = [],
)

cc_library(
    name = "catch_intent",
    srcs = ["catch_intent.cpp"],
    hdrs = ["catch_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/ai/primitive:catch_primitive",
    ],
)

cc_library(
    name = "chip_intent",
    srcs = ["chip_intent.cpp"],
    hdrs = ["chip_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/ai/primitive:chip_primitive",
    ],
)

cc_library(
    name = "direct_velocity_intent",
    srcs = ["direct_velocity_intent.cpp"],
    hdrs = ["direct_velocity_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/ai/primitive:direct_velocity_primitive",
    ],
)

cc_library(
    name = "direct_wheels_intent",
    srcs = ["direct_wheels_intent.cpp"],
    hdrs = ["direct_wheels_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/ai/primitive:direct_wheels_primitive",
    ],
)

cc_library(
    name = "dribble_intent",
    srcs = ["dribble_intent.cpp"],
    hdrs = ["dribble_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/ai/primitive:dribble_primitive",
    ],
)

cc_library(
    name = "kick_intent",
    srcs = ["kick_intent.cpp"],
    hdrs = ["kick_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/ai/primitive:kick_primitive",
    ],
)

cc_library(
    name = "move_intent",
    srcs = ["move_intent.cpp"],
    hdrs = ["move_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/ai/primitive:move_primitive",
    ],
)

cc_library(
    name = "movespin_intent",
    srcs = ["movespin_intent.cpp"],
    hdrs = ["movespin_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/ai/primitive:movespin_primitive",
    ],
)

cc_library(
    name = "pivot_intent",
    srcs = ["pivot_intent.cpp"],
    hdrs = ["pivot_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/ai/primitive:pivot_primitive",
    ],
)

cc_library(
    name = "stop_intent",
    srcs = ["stop_intent.cpp"],
    hdrs = ["stop_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/ai/primitive:stop_primitive",
    ],
)

cc_library(
    name = "all_intents",
    deps = [
        ":catch_intent",
        ":chip_intent",
        ":direct_velocity_intent",
        ":direct_wheels_intent",
        ":dribble_intent",
        ":kick_intent",
        ":move_intent",
        ":movespin_intent",
        ":pivot_intent",
        ":stop_intent",
    ],
)

cc_library(
    name = "intent",
    srcs = ["intent.cpp"],
    hdrs = ["intent.h"],
    deps = [":avoid_area"],
)

cc_library(
    name = "intent_visitor",
    hdrs = ["intent_visitor.h"],
    deps = [":intent"],
)
