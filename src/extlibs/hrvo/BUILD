package(default_visibility = ["//visibility:public"])

cc_test(
    name = "hrvo_test",
    srcs = [
        "hrvo_test.cpp",
    ],
    deps = [
        ":hrvo",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "hrvo",
    srcs = [
        "agent.cpp",
        "goal.cpp",
        "kd_tree.cpp",
        "simulator.cpp",
        "vector2.cpp",
    ],
    hdrs = [
        "agent.h",
        "goal.h",
        "kd_tree.h",
        "simulator.h",
        "vector2.h",
    ],
    visibility = ["//visibility:public"],
)
