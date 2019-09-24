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

cc_test(
    name = "chip_action_test",
    srcs = ["chip_action_test.cpp"],
    deps = [
        ":chip_action",
        "@gtest//:gtest_main",
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

cc_test(
    name = "dribble_action_test",
    srcs = ["dribble_action_test.cpp"],
    deps = [
        ":dribble_action",
        "@gtest//:gtest_main",
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

cc_test(
    name = "kick_action_test",
    srcs = ["kick_action_test.cpp"],
    deps = [
        ":kick_action",
        "@gtest//:gtest_main",
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

cc_test(
    name = "move_action_test",
    srcs = ["move_action_test.cpp"],
    deps = [
        ":move_action",
        "@gtest//:gtest_main",
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

cc_test(
    name = "movespin_action_test",
    srcs = ["movespin_action_test.cpp"],
    deps = [
        ":movespin_action",
        "@gtest//:gtest_main",
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

cc_test(
    name = "pivot_action_test",
    srcs = ["pivot_action_test.cpp"],
    deps = [
        ":pivot_action",
        "@gtest//:gtest_main",
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

cc_test(
    name = "stop_action_test",
    srcs = ["stop_action_test.cpp"],
    deps = [
        ":stop_action",
        "@gtest//:gtest_main",
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
        "@boost//:coroutine2",
    ],
)

cc_test(
    name = "action_test",
    srcs = ["action_test.cpp"],
    deps = [
        ":action",
        "//software/ai/hl/stp/action/test_actions:move_test_action",
        "//software/ai/intent:move_intent",
        "@gtest//:gtest_main",
    ],
)
