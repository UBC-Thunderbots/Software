package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ball",
    srcs = ["ball.cpp"],
    hdrs = ["ball.h"],
    deps = [
        "//software/geom:all",
        "//software/util/time:timestamp",
        "@boost//:circular_buffer",
    ],
)

cc_library(
    name = "field",
    srcs = ["field.cpp"],
    hdrs = ["field.h"],
    deps = [
        "//software/geom:all",
        "//software/util/time:timestamp",
        "@boost//:circular_buffer",
    ],
)

cc_library(
    name = "game_state",
    srcs = ["game_state.cpp"],
    hdrs = ["game_state.h"],
    deps = [
        ":ball",
        ":refbox_constants",
        "@g3log//:all",
    ],
)

cc_library(
    name = "refbox_constants",
    srcs = ["refbox_constants.cpp"],
    hdrs = ["refbox_constants.h"],
    deps = [],
)

cc_library(
    name = "robot",
    srcs = ["robot.cpp"],
    hdrs = ["robot.h"],
    deps = [
        ":robot_capabilities",
        "//software/geom:all",
        "//software/util/time:timestamp",
        "@boost//:circular_buffer",
    ],
)

cc_library(
    name = "robot_capabilities",
    hdrs = ["robot_capabilities.h"],
    deps = [],
)

cc_library(
    name = "team",
    srcs = ["team.cpp"],
    hdrs = ["team.h"],
    deps = [":robot"],
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
