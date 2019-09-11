package(default_visibility = ["//visibility:public"])

cc_library(
    name = "evaluation",
    srcs = ["evaluation.cpp"],
    hdrs = ["evaluation.h"],
    deps = [
        ":pass",
        "//software/ai/evaluation:pass",
        "//software/ai/world",
        "//software/geom:all",
        "//software/util/math:math_functions",
    ],
)

cc_library(
    name = "pass",
    srcs = ["pass.cpp"],
    hdrs = ["pass.h"],
    deps = [
        "//software/ai/world:field",
        "//software/geom:all",
        "//software/util/time:timestamp",
    ],
)

cc_library(
    name = "pass_generator",
    srcs = ["pass_generator.cpp"],
    hdrs = ["pass_generator.h"],
    deps = [
        ":evaluation",
        ":pass",
        "//software/ai/world",
        "//software/util/canvas_messenger",
        "//software/util/optimization:gradient_descent",
    ],
)
