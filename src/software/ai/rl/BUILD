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
        ":torch",
        "//software/ai/rl/replay:transition",
    ],
)
