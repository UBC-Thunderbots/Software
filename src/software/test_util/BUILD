package(default_visibility = ["//visibility:public"])

cc_library(
    name = "test_util",
    testonly = True,
    srcs = ["test_util.cpp"],
    hdrs = ["test_util.h"],
    deps = [
        "//shared:constants",
        "//software/new_geom:geom_constants",
        "//software/new_geom:point",
        "//software/new_geom:polygon",
        "//software/new_geom/util:almost_equal",
        "//software/world",
    ],
)

cc_test(
    name = "test_util_test",
    srcs = ["test_util_test.cpp"],
    deps = [
        ":test_util",
        "@gtest//:gtest_main",
    ],
)
