package(default_visibility = ["//visibility:public"])

cc_library(
    name = "possession_tracker",
    srcs = ["possession_tracker.cpp"],
    hdrs = ["possession_tracker.h"],
    deps = [
        "//software/ai/evaluation:possession",
        "//software/geom/algorithms",
        "//software/time:duration",
        "//software/time:timestamp",
        "//software/world",
    ],
)

cc_test(
    name = "possession_tracker_test",
    srcs = ["possession_tracker_test.cpp"],
    deps = [
        ":possession_tracker",
        "//shared:constants",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
