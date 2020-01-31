package(default_visibility = ["//visibility:public"])

cc_library(
    name = "accurate_shoot",
    srcs = ["accurate_shoot.c"],
    hdrs = ["accurate_shoot.h"],
    deps = [
        ":primitive",
        "//firmware/main/app/control",
        "//firmware/main/app/control:bangbang",
        "//firmware/main/shared:physics",
        "//firmware/main/shared:util",
    ],
)

cc_library(
    name = "catch",
    srcs = ["catch.c"],
    hdrs = ["catch.h"],
    deps = [
        ":primitive",
        "//firmware/main/app/control",
        "//firmware/main/app/control:bangbang",
        "//firmware/main/shared:physics",
        "//firmware/main/shared:util",
    ],
)

cc_library(
    name = "direct_velocity",
    srcs = ["direct_velocity.c"],
    hdrs = ["direct_velocity.h"],
    deps = [
        ":primitive",
        "//firmware/main/app/control",
    ],
)

cc_library(
    name = "direct_wheels",
    srcs = ["direct_wheels.c"],
    hdrs = ["direct_wheels.h"],
    deps = [
        ":primitive",
    ],
)

cc_library(
    name = "dribble",
    srcs = ["dribble.c"],
    hdrs = ["dribble.h"],
    deps = [
        ":primitive",
        "//firmware/main/app/control",
        "//firmware/main/app/control:bangbang",
        "//firmware/main/shared:physics",
    ],
)

cc_library(
    name = "imu_test",
    srcs = ["imu_test.c"],
    hdrs = ["imu_test.h"],
    deps = [
        ":primitive",
        "//firmware/main/app/control",
        "//firmware/main/app/control:bangbang",
        "//firmware/main/shared:physics",
    ],
)

cc_library(
    name = "move",
    srcs = ["move.c"],
    hdrs = ["move.h"],
    deps = [
        ":primitive",
        "//firmware/main/app/control",
        "//firmware/main/app/control:bangbang",
        "//firmware/main/app/control:physbot",
        "//firmware/main/shared:physics",
        "//firmware/main/shared:util",
        "//shared:constants",
        "//shared:robot_constants",
    ],
)

cc_library(
    name = "pivot",
    srcs = ["pivot.c"],
    hdrs = ["pivot.h"],
    deps = [
        ":primitive",
        "//firmware/main/app/control",
        "//firmware/main/app/control:bangbang",
        "//firmware/main/shared:util",
    ],
)

cc_library(
    name = "primitive",
    srcs = ["primitive.c"],
    hdrs = ["primitive.h"],
    deps = [
        "//firmware/main/app/world:firmware_world",
    ],
)

cc_library(
    name = "primitive_manager",
    srcs = ["primitive_manager.c"],
    hdrs = ["primitive_manager.h"],
    deps = [
        ":catch",
        ":direct_velocity",
        ":direct_wheels",
        ":dribble",
        ":imu_test",
        ":move",
        ":pivot",
        ":primitive",
        ":shoot",
        ":spin",
        ":stop",
        "//firmware/main/app/world:firmware_world",
    ] + select({
        "//cc_toolchain:cpu_stm32f4": ["//firmware:main_freertos"],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "shoot",
    srcs = ["shoot.c"],
    hdrs = ["shoot.h"],
    deps = [
        ":primitive",
        "//firmware/main/app/control",
        "//firmware/main/app/control:bangbang",
        "//firmware/main/app/control:physbot",
        "//firmware/main/shared:physics",
        "//firmware/main/shared:util",
    ],
)

cc_library(
    name = "spin",
    srcs = ["spin.c"],
    hdrs = ["spin.h"],
    deps = [
        ":primitive",
        "//firmware/main/app/control",
        "//firmware/main/app/control:bangbang",
        "//firmware/main/shared:physics",
    ],
)

cc_library(
    name = "stop",
    srcs = ["stop.c"],
    hdrs = ["stop.h"],
    deps = [
        ":primitive",
    ],
)
