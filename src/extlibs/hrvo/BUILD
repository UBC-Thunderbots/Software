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
        "path.cpp",
        "path_point.cpp",
        "hrvo_agent.cpp",
        "kd_tree.cpp",
        "linear_velocity_agent.cpp",
        "simulator.cpp",
        "vector2.cpp",
    ],
    hdrs = [
        "agent.h",
        "goal.h",
        "path.h",
        "path_point.h",
        "hrvo_agent.h",
        "kd_tree.h",
        "linear_velocity_agent.h",
        "simulator.h",
        "vector2.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//proto/primitive:primitive_msg_factory",
        "//software/world",
    ],
)
