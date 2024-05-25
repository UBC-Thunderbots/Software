package(default_visibility = ["//visibility:public"])

cc_library(
    name = "feature_extractor",
    hdrs = ["feature_extractor.hpp"],
    deps = [
        "//software/util/make_enum:reflective_enum",
    ]
)

cc_library(
    name = "q_function",
    hdrs = ["q_function.hpp"],
    deps = [
        "//software/util/make_enum:reflective_enum",
    ],
)

cc_library(
    name = "linear_q_function",
    hdrs = ["linear_q_function.hpp"],
    deps = [
        ":q_function",
        ":feature_extractor",
        "@eigen"
    ]
)
