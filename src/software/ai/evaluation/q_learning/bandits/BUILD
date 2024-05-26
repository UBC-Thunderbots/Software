package(default_visibility = ["//visibility:public"])

cc_library(
    name = "action_selection_strategy",
    hdrs = ["action_selection_strategy.hpp"],
    deps = [
        "//software/ai/evaluation/q_learning:q_function",
    ],
)

cc_library(
    name = "epsilon_greedy_strategy",
    hdrs = ["epsilon_greedy_strategy.hpp"],
    deps = [
        ":action_selection_strategy",
    ],
)

cc_library(
    name = "softmax_strategy",
    hdrs = ["softmax_strategy.hpp"],
    deps = [
        ":action_selection_strategy",
        "@eigen",
    ],
)
