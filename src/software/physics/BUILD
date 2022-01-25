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
        "//software/time:duration",
        "@eigen",
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
