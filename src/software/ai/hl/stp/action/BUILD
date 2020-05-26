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
    name = "intercept_ball_action",
    srcs = ["intercept_ball_action.cpp"],
    hdrs = ["intercept_ball_action.h"],
    deps = [
        ":action",
        "//shared:constants",
        "//software/ai/evaluation:intercept",
        "//software/ai/evaluation:robot",
        "//software/ai/intent:move_intent",
        "//software/geom",
        "//software/logger",
        "//software/new_geom/util:closest_point",
        "//software/new_geom/util:distance",
        "//software/new_geom/util:intersection",
        "//software/world:ball",
        "//software/world:field",
    ],
)

cc_test(
    name = "intercept_ball_action_test",
    srcs = ["intercept_ball_action_test.cpp"],
    deps = [
        ":intercept_ball_action",
        "//software/ai/intent:move_intent",
        "//software/test_util",
        "//software/world:ball",
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
        "//software/world:ball",
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
        "//software/logger",
        "//software/parameter:dynamic_parameters",
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
    name = "all_actions",
    srcs = ["all_actions.h"],
    deps = [
        ":chip_action",
        ":dribble_action",
        ":intercept_ball_action",
        ":kick_action",
        ":move_action",
        ":movespin_action",
        ":pivot_action",
        ":stop_action",
    ],
)

cc_library(
    name = "action",
    srcs = ["action.cpp"],
    hdrs = ["action.h"],
    deps = [
        ":mutable_action_visitor",
        "//software/ai/intent",
        "//software/world:ball",
        "//software/world:robot",
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

cc_library(
    name = "mutable_action_visitor",
    hdrs = ["mutable_action_visitor.h"],
)

cc_library(
    name = "action_world_params_update_visitor",
    srcs = ["action_world_params_update_visitor.cpp"],
    hdrs = ["action_world_params_update_visitor.h"],
    deps = [
        ":all_actions",
        ":mutable_action_visitor",
        "//software/world",
    ],
)
