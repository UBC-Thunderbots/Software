package(default_visibility = ["//visibility:public"])

cc_library(
    name = "direct_primitive_intent",
    srcs = ["direct_primitive_intent.cpp"],
    hdrs = ["direct_primitive_intent.h"],
    deps = [
        ":intent",
    ],
)

cc_library(
    name = "navigating_intent",
    srcs = ["navigating_intent.cpp"],
    hdrs = ["navigating_intent.h"],
    deps = [
        "navigating_intent_visitor",
        ":intent",
    ],
)

cc_library(
    name = "chip_intent",
    srcs = ["chip_intent.cpp"],
    hdrs = ["chip_intent.h"],
    deps = [
        ":direct_primitive_intent",
    ],
)

cc_test(
    name = "chip_intent_test",
    srcs = ["chip_intent_test.cpp"],
    deps = [
        ":chip_intent",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "kick_intent",
    srcs = ["kick_intent.cpp"],
    hdrs = ["kick_intent.h"],
    deps = [
        ":direct_primitive_intent",
    ],
)

cc_test(
    name = "kick_intent_test",
    srcs = ["kick_intent_test.cpp"],
    deps = [
        ":kick_intent",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "move_intent",
    srcs = ["move_intent.cpp"],
    hdrs = ["move_intent.h"],
    deps = [
        ":navigating_intent",
        "//software/geom:angle",
        "//software/geom:point",
        "//software/util/make_enum",
    ],
)

cc_test(
    name = "move_intent_test",
    srcs = ["move_intent_test.cpp"],
    deps = [
        ":move_intent",
        "//shared/test_util:tbots_gtest_main",
        "//software/util/typename",
    ],
)

cc_library(
    name = "stop_intent",
    srcs = ["stop_intent.cpp"],
    hdrs = ["stop_intent.h"],
    deps = [
        ":direct_primitive_intent",
    ],
)

cc_test(
    name = "stop_intent_test",
    srcs = ["stop_intent_test.cpp"],
    deps = [
        ":stop_intent",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "all_intents",
    hdrs = ["all_intents.h"],
    deps = [
        ":chip_intent",
        ":direct_primitive_intent",
        ":kick_intent",
        ":move_intent",
        ":navigating_intent",
        ":stop_intent",
    ],
)

cc_library(
    name = "intent",
    srcs = ["intent.cpp"],
    hdrs = ["intent.h"],
    deps = [
        "intent_visitor",
        "//shared/proto:tbots_cc_proto",
        "//software/ai/motion_constraint",
        "//software/geom:angle",
        "//software/geom:point",
        "//software/logger",
        "//software/proto/primitive:primitive_msg_factory",
    ],
)

cc_test(
    name = "intent_test",
    srcs = ["intent_test.cpp"],
    deps = [
        ":intent",
        ":stop_intent",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "intent_visitor",
    hdrs = ["intent_visitor.h"],
)

cc_library(
    name = "navigating_intent_visitor",
    hdrs = ["navigating_intent_visitor.h"],
)
