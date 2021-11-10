package(default_visibility = ["//visibility:public"])

cc_library(
    name = "physics",
    srcs = ["physics.c"],
    hdrs = ["physics.h"],
    deps = [
        ":physics_linear_alebra",
        "//firmware/shared:util",
        "//firmware/shared/math:tbots_math",
        "//shared:constants",
    ],
)

cc_test(
    name = "physics_test",
    srcs = ["physics_test.cpp"],
    deps = [
        ":physics",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_library(
    name = "physics_linear_alebra",
    srcs = ["physics_linear_algebra.c"],
    hdrs = ["physics_linear_algebra.h"],
    visibility = ["//visibility:private"],
    deps = [],
)

cc_library(
    name = "util",
    srcs = ["util.c"],
    hdrs = ["util.h"],
    deps = [
        "//firmware/shared/math:tbots_math",
    ],
)

cc_library(
    name = "circular_buffer",
    srcs = ["circular_buffer.c"],
    hdrs = ["circular_buffer.h"],
    deps = [
        "//shared:constants",
    ],
)

cc_test(
    name = "circular_buffer_test",
    srcs = ["circular_buffer_test.cpp"],
    deps = [
        ":circular_buffer",
        "//shared/test_util:tbots_gtest_main",
    ],
)
