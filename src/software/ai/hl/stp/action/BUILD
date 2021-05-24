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
        "//software/geom:polygon",
        "//software/geom:triangle",
        "//software/geom/algorithms",
        "//software/world:ball",
    ],
)

cc_test(
    name = "chip_action_test",
    srcs = ["chip_action_test.cpp"],
    deps = [
        ":chip_action",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_library(
    name = "intercept_ball_action",
    srcs = ["intercept_ball_action.cpp"],
    hdrs = ["intercept_ball_action.h"],
    deps = [
        ":action",
        ":move_action",
        "//shared:constants",
        "//software/ai/evaluation:intercept",
        "//software/ai/intent:move_intent",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/world:ball",
        "//software/world:field",
    ],
)

cc_test(
    name = "intercept_ball_action_test",
    srcs = ["intercept_ball_action_test.cpp"],
    deps = [
        ":intercept_ball_action",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/intent:move_intent",
        "//software/geom:segment",
        "//software/geom/algorithms",
        "//software/test_util",
        "//software/world:ball",
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
        "//software/geom:polygon",
        "//software/geom/algorithms",
        "//software/world:ball",
    ],
)

cc_test(
    name = "kick_action_test",
    srcs = ["kick_action_test.cpp"],
    deps = [
        ":kick_action",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
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
        "//shared/test_util:tbots_gtest_main",
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
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "all_actions",
    srcs = ["all_actions.h"],
    deps = [
        ":chip_action",
        ":intercept_ball_action",
        ":kick_action",
        ":move_action",
        ":stop_action",
    ],
)

cc_library(
    name = "action",
    srcs = ["action.cpp"],
    hdrs = ["action.h"],
    deps = [
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
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/action/test_actions:move_test_action",
        "//software/ai/intent:move_intent",
    ],
)
