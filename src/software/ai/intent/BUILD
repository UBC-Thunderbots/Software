package(default_visibility = ["//visibility:public"])

cc_library(
    name = "catch_intent",
    srcs = ["catch_intent.cpp"],
    hdrs = ["catch_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/primitive:catch_primitive",
    ],
)

cc_test(
    name = "catch_intent_test",
    srcs = ["catch_intent_test.cpp"],
    deps = [
        ":catch_intent",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "chip_intent",
    srcs = ["chip_intent.cpp"],
    hdrs = ["chip_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/primitive:chip_primitive",
    ],
)

cc_test(
    name = "chip_intent_test",
    srcs = ["chip_intent_test.cpp"],
    deps = [
        ":chip_intent",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "direct_velocity_intent",
    srcs = ["direct_velocity_intent.cpp"],
    hdrs = ["direct_velocity_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/primitive:direct_velocity_primitive",
    ],
)

cc_test(
    name = "direct_velocity_intent_test",
    srcs = ["direct_velocity_intent_test.cpp"],
    deps = [
        ":direct_velocity_intent",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "direct_wheels_intent",
    srcs = ["direct_wheels_intent.cpp"],
    hdrs = ["direct_wheels_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/primitive:direct_wheels_primitive",
    ],
)

cc_test(
    name = "direct_wheels_intent_test",
    srcs = ["direct_wheels_intent_test.cpp"],
    deps = [
        ":direct_wheels_intent",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "dribble_intent",
    srcs = ["dribble_intent.cpp"],
    hdrs = ["dribble_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/primitive:dribble_primitive",
    ],
)

cc_test(
    name = "dribble_intent_test",
    srcs = ["dribble_intent_test.cpp"],
    deps = [
        ":dribble_intent",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "kick_intent",
    srcs = ["kick_intent.cpp"],
    hdrs = ["kick_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/primitive:kick_primitive",
    ],
)

cc_test(
    name = "kick_intent_test",
    srcs = ["kick_intent_test.cpp"],
    deps = [
        ":kick_intent",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "move_intent",
    srcs = ["move_intent.cpp"],
    hdrs = ["move_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/primitive:move_primitive",
    ],
)

cc_test(
    name = "move_intent_test",
    srcs = ["move_intent_test.cpp"],
    deps = [
        ":move_intent",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "movespin_intent",
    srcs = ["movespin_intent.cpp"],
    hdrs = ["movespin_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/primitive:movespin_primitive",
    ],
)

cc_test(
    name = "movespin_intent_test",
    srcs = ["movespin_intent_test.cpp"],
    deps = [
        ":movespin_intent",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "pivot_intent",
    srcs = ["pivot_intent.cpp"],
    hdrs = ["pivot_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/primitive:pivot_primitive",
    ],
)

cc_test(
    name = "pivot_intent_test",
    srcs = ["pivot_intent_test.cpp"],
    deps = [
        ":pivot_intent",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "stop_intent",
    srcs = ["stop_intent.cpp"],
    hdrs = ["stop_intent.h"],
    deps = [
        ":intent",
        ":intent_visitor",
        "//software/primitive:stop_primitive",
    ],
)

cc_test(
    name = "stop_intent_test",
    srcs = ["stop_intent_test.cpp"],
    deps = [
        ":stop_intent",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "all_intents",
    hdrs = ["all_intents.h"],
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
    deps = [
        "//software/ai/motion_constraint",
        "//software/logger",
    ],
)

cc_test(
    name = "intent_test",
    srcs = ["intent_test.cpp"],
    deps = [
        ":intent",
        ":move_intent",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "intent_visitor",
    hdrs = ["intent_visitor.h"],
    deps = [":intent"],
)
