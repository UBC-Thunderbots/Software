package(default_visibility = ["//visibility:public"])

cc_library(
    name = "exploration_strategy",
    hdrs = ["exploration_strategy.hpp"],
    deps = [
        "//software/util/make_enum:make_enum",
        "//software/ai/rl:torch",
    ],
)

cc_library(
    name = "epsilon_greedy_strategy",
    srcs = ["epsilon_greedy_strategy.hpp"],
    deps = [":exploration_strategy"],
)
