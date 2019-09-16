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

cc_test(
    name = "evaluation_test",
    srcs = ["evaluation_test.cpp"],
    deps = [
        ":evaluation",
        "//software/test/test_util",
        "//software/util/math:math_functions",
        "@gtest//:gtest_main",
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

cc_test(
    name = "pass_test",
    srcs = ["pass_test.cpp"],
    deps = [
        ":pass",
        "//software/test/test_util",
        "@gtest//:gtest_main",
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

# TODO: (Issue #655) un-comment this and fix the flaky tests in it
#cc_test(
#    name = "pass_generator_test",
#    srcs = ["pass_generator_test.cpp"],
#    deps = [
#        ":pass_generator",
#        "//software/test/test_util",
#        "@gtest//:gtest_main",
#    ],
#)
