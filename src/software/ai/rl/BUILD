package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dqn",
    hdrs = ["dqn.hpp"],
    deps = [
        ":replay_buffer",
        "@pytorch",
    ],
)

cc_library(
    name = "replay_buffer",
    hdrs = ["replay_buffer.hpp"],
    deps = ["//software/logger"],
)

cc_test(
    name = "dqn_test",
    srcs = ["dqn_test.cpp"],
    deps = [
        ":dqn",
        "//shared/test_util:tbots_gtest_main",
    ],
)
