package(default_visibility = ["//visibility:public"])

cc_library(
    name = "torch",
    hdrs = ["torch.h"],
    deps = ["@pytorch"],
)

cc_library(
    name = "dqn",
    hdrs = ["dqn.hpp"],
    deps = [
        ":replay_buffer",
        ":torch",
    ],
)

cc_library(
    name = "replay_buffer",
    hdrs = ["replay_buffer.hpp"],
    deps = ["//software/logger"],
)
