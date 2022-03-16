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
        "hrvo_agent.cpp",
        "kd_tree.cpp",
        "linear_velocity_agent.cpp",
        "simulator.cpp",
    ],
    hdrs = [
        "agent.h",
        "goal.h",
        "hrvo_agent.h",
        "kd_tree.h",
        "linear_velocity_agent.h",
        "simulator.h",
    ],
    deps = [
        "//proto/primitive:primitive_msg_factory",
        "//software/geom:vector",
        "//software/world",
    ],
)
