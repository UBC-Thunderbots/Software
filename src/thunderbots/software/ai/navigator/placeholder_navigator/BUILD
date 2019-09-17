package(default_visibility = ["//visibility:public"])

cc_library(
    name = "placeholder_navigator",
    srcs = ["placeholder_navigator.cpp"],
    hdrs = ["placeholder_navigator.h"],
    deps = [
        "//software/ai/intent",
        "//software/ai/intent:all_intents",
        "//software/ai/intent:intent_visitor",
        "//software/ai/navigator",
        "//software/ai/primitive:all_primitives",
        "//software/ai/world",
    ],
)

cc_test(
    name = "placeholder_navigator_test",
    srcs = ["placeholder_navigator_test.cpp"],
    deps = [
        ":placeholder_navigator",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
