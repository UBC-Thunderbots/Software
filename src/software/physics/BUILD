package(default_visibility = ["//visibility:public"])

cc_library(
    name = "physics",
    srcs = [
        "physics.cpp",
    ],
    hdrs = [
        "physics.h",
    ],
    deps = [
        "//software/geom:point",
    ],
)

cc_test(
    name = "physics_test",
    srcs = ["physics_test.cpp"],
    deps = [
        ":physics",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
