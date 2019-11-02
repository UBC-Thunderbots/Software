package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ball",
    srcs = ["ball.cpp"],
    hdrs = ["ball.h"],
    deps = [
        "//software/geom",
        "//software/util/time:timestamp",
        "@boost//:circular_buffer",
    ],
)

cc_test(
    name = "ball_test",
    srcs = ["ball_test.cpp"],
    deps = [
        ":ball",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "field",
    srcs = ["field.cpp"],
    hdrs = ["field.h"],
    deps = [
        "//shared:constants",
        "//software/geom",
        "//software/util/time:timestamp",
        "@boost//:circular_buffer",
    ],
)

cc_test(
    name = "field_test",
    srcs = ["field_test.cpp"],
    deps = [
        ":field",
        "//shared:constants",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "game_state",
    srcs = ["game_state.cpp"],
    hdrs = ["game_state.h"],
    deps = [
        ":ball",
        ":refbox_constants",
        "@g3log",
    ],
)

cc_test(
    name = "game_state_test",
    srcs = ["game_state_test.cpp"],
    deps = [
        ":game_state",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "refbox_constants",
    srcs = ["refbox_constants.cpp"],
    hdrs = ["refbox_constants.h"],
    deps = [],
)

cc_test(
    name = "refbox_constants_test",
    srcs = ["refbox_constants_test.cpp"],
    deps = [
        ":refbox_constants",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "robot",
    srcs = ["robot.cpp"],
    hdrs = ["robot.h"],
    deps = [
        ":robot_capabilities",
        "//software/geom",
        "//software/util/time:timestamp",
        "@boost//:circular_buffer",
    ],
)

cc_test(
    name = "robot_test",
    srcs = ["robot_test.cpp"],
    deps = [
        ":robot",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "robot_capabilities",
    hdrs = ["robot_capabilities.h"],
    deps = [],
)

cc_test(
    name = "robot_capabilities_test",
    srcs = ["robot_capabilities_test.cpp"],
    deps = [
        "robot_capabilities",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "team",
    srcs = ["team.cpp"],
    hdrs = ["team.h"],
    deps = [":robot"],
)

cc_test(
    name = "team_test",
    srcs = ["team_test.cpp"],
    deps = [
        ":team",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "world",
    srcs = ["world.cpp"],
    hdrs = ["world.h"],
    deps = [
        ":ball",
        ":field",
        ":game_state",
        ":robot",
        ":team",
        "//software/util/parameter:dynamic_parameters",
    ],
)

cc_test(
    name = "world_test",
    srcs = ["world_test.cpp"],
    deps = [
        ":world",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
