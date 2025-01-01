package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transition",
    hdrs = ["transition.hpp"],
    deps = ["//software/util/make_enum"],
)

cc_library(
    name = "uniform_replay_buffer",
    hdrs = ["uniform_replay_buffer.hpp"],
    deps = [
        ":transition",
        "//software/logger",
    ],
)

cc_library(
    name = "prioritized_replay_buffer",
    hdrs = ["prioritized_replay_buffer.hpp"],
    deps = [
        ":sum_tree",
        ":transition",
        "//software/ai/rl:torch",
    ],
)

cc_library(
    name = "sum_tree",
    hdrs = ["sum_tree.hpp"],
    deps = ["//software/logger"],
)

cc_test(
    name = "sum_tree_test",
    srcs = ["sum_tree_test.cpp"],
    deps = [
        ":sum_tree",
        "//shared/test_util:tbots_gtest_main",
    ],
)
