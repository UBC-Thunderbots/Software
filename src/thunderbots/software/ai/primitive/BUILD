package(default_visibility = ["//visibility:public"])

cc_library(
    name = "catch_primitive",
    srcs = ["catch_primitive.cpp"],
    hdrs = ["catch_primitive.h"],
    deps = [
        ":primitive",
        ":primitive_visitor",
        "//software/geom:all",
    ],
)

cc_library(
    name = "chip_primitive",
    srcs = ["chip_primitive.cpp"],
    hdrs = ["chip_primitive.h"],
    deps = [
        ":primitive",
        ":primitive_visitor",
        "//software/geom:all",
    ],
)

cc_library(
    name = "direct_velocity_primitive",
    srcs = ["direct_velocity_primitive.cpp"],
    hdrs = ["direct_velocity_primitive.h"],
    deps = [
        ":primitive",
        ":primitive_visitor",
        "//software/geom:all",
    ],
)

cc_library(
    name = "direct_wheels_primitive",
    srcs = ["direct_wheels_primitive.cpp"],
    hdrs = ["direct_wheels_primitive.h"],
    deps = [
        ":primitive",
        ":primitive_visitor",
        "//software/geom:all",
    ],
)

cc_library(
    name = "dribble_primitive",
    srcs = ["dribble_primitive.cpp"],
    hdrs = ["dribble_primitive.h"],
    deps = [
        ":primitive",
        ":primitive_visitor",
        "//software/geom:all",
    ],
)

cc_library(
    name = "kick_primitive",
    srcs = ["kick_primitive.cpp"],
    hdrs = ["kick_primitive.h"],
    deps = [
        ":primitive",
        ":primitive_visitor",
        "//software/geom:all",
    ],
)

cc_library(
    name = "move_primitive",
    srcs = ["move_primitive.cpp"],
    hdrs = ["move_primitive.h"],
    deps = [
        ":primitive",
        ":primitive_visitor",
        "//software/geom:all",
    ],
)

cc_library(
    name = "movespin_primitive",
    srcs = ["movespin_primitive.cpp"],
    hdrs = ["movespin_primitive.h"],
    deps = [
        ":primitive",
        ":primitive_visitor",
        "//software/geom:all",
    ],
)

cc_library(
    name = "pivot_primitive",
    srcs = ["pivot_primitive.cpp"],
    hdrs = ["pivot_primitive.h"],
    deps = [
        ":primitive",
        ":primitive_visitor",
        "//shared:constants",
        "//software/geom:all",
    ],
)

cc_library(
    name = "stop_primitive",
    srcs = ["stop_primitive.cpp"],
    hdrs = ["stop_primitive.h"],
    deps = [
        ":primitive",
        ":primitive_visitor",
        "//software/geom:all",
    ],
)

cc_library(
    name = "all_primitives",
    deps = [
        ":catch_primitive",
        ":chip_primitive",
        ":direct_velocity_primitive",
        ":direct_wheels_primitive",
        ":dribble_primitive",
        ":kick_primitive",
        ":move_primitive",
        ":movespin_primitive",
        ":pivot_primitive",
        ":stop_primitive",
    ],
)

cc_library(
    name = "primitive_visitor",
    hdrs = ["primitive_visitor.h"],
    deps = [],
)

cc_library(
    name = "primitive",
    hdrs = ["primitive.h"],
    deps = [],
)
