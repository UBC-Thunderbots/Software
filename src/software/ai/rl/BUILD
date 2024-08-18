package(default_visibility = ["//visibility:public"])

cc_library(
    name = "feature_extractor",
    hdrs = ["feature_extractor.hpp"],
    deps = [
        "//software/util/make_enum",
    ],
)

cc_library(
    name = "q_function",
    hdrs = ["q_function.hpp"],
    deps = [
        "//software/util/make_enum",
    ],
)

cc_library(
    name = "linear_q_function",
    hdrs = ["linear_q_function.hpp"],
    deps = [
        ":feature_extractor",
        ":q_function",
        "//proto:tbots_cc_proto",
        "@csv",
        "@eigen",
    ],
)

cc_library(
    name = "q_policy",
    hdrs = ["q_policy.hpp"],
    deps = [
        ":feature_extractor",
        ":q_function",
        "//proto:tbots_cc_proto",
        "//software/ai/rl/action_selection:action_selection_strategy",
        "@eigen",
    ],
)
