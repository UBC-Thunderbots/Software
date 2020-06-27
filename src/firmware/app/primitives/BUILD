package(default_visibility = ["//visibility:public"])

cc_library(
    name = "accurate_shoot",
    srcs = ["accurate_shoot_primitive.c"],
    hdrs = ["accurate_shoot_primitive.h"],
    deps = [
        ":primitive",
        "//firmware/app/control",
        "//firmware/app/control:bangbang",
        "//firmware/shared:physics",
        "//firmware/shared:util",
    ],
)

cc_library(
    name = "catch",
    srcs = ["catch_primitive.c"],
    hdrs = ["catch_primitive.h"],
    deps = [
        ":primitive",
        "//firmware/app/control",
        "//firmware/app/control:bangbang",
        "//firmware/shared:physics",
        "//firmware/shared:util",
    ],
)

cc_library(
    name = "direct_velocity",
    srcs = ["direct_velocity_primitive.c"],
    hdrs = ["direct_velocity_primitive.h"],
    deps = [
        ":primitive",
        "//firmware/app/control",
    ],
)

cc_library(
    name = "direct_wheels",
    srcs = ["direct_wheels_primitive.c"],
    hdrs = ["direct_wheels_primitive.h"],
    deps = [
        ":primitive",
    ],
)

cc_library(
    name = "dribble",
    srcs = ["dribble_primitive.c"],
    hdrs = ["dribble_primitive.h"],
    deps = [
        ":primitive",
        "//firmware/app/control",
        "//firmware/app/control:bangbang",
        "//firmware/shared:physics",
    ],
)

cc_library(
    name = "imu_test",
    srcs = ["imu_test_primitive.c"],
    hdrs = ["imu_test_primitive.h"],
    deps = [
        ":primitive",
        "//firmware/app/control",
        "//firmware/app/control:bangbang",
        "//firmware/shared:physics",
    ],
)

cc_library(
    name = "move",
    srcs = ["move_primitive.c"],
    hdrs = ["move_primitive.h"],
    deps = [
        ":primitive",
        "//firmware/app/control",
        "//firmware/app/control:bangbang",
        "//firmware/app/control:physbot",
        "//firmware/shared:physics",
        "//firmware/shared:util",
        "//shared:constants",
        "//shared:robot_constants",
    ],
)

cc_library(
    name = "pivot",
    srcs = ["pivot_primitive.c"],
    hdrs = ["pivot_primitive.h"],
    deps = [
        ":primitive",
        "//firmware/app/control",
        "//firmware/app/control:bangbang",
        "//firmware/shared:util",
    ],
)

cc_library(
    name = "primitive",
    srcs = ["primitive.c"],
    hdrs = ["primitive.h"],
    deps = [
        "//firmware/app/world:firmware_world",
    ],
)

cc_library(
    name = "primitive_manager",
    srcs = ["primitive_manager.c"],
    hdrs = ["primitive_manager.h"],
    copts = select({
        # TODO: this should not be dependent on freertos at all
        "//cc_toolchain:cpu_stm32f4": [
            "-isystem firmware/boards/shared/legacy_freertos/include",
            "-isystem firmware/boards/shared/legacy_stm32lib/include",
        ],
        "//conditions:default": [],
    }),
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
        "//firmware/app/world:firmware_world",
    ] + select({
        # TODO: this should not be dependent on freertos at all
        "//cc_toolchain:cpu_stm32f4": ["//firmware/boards/legacy_robot_stm32f4:legacy_shared_libs"],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "shoot",
    srcs = ["shoot_primitive.c"],
    hdrs = ["shoot_primitive.h"],
    deps = [
        ":primitive",
        "//firmware/app/control",
        "//firmware/app/control:bangbang",
        "//firmware/app/control:physbot",
        "//firmware/shared:physics",
        "//firmware/shared:util",
    ],
)

cc_library(
    name = "spin",
    srcs = ["spin_primitive.c"],
    hdrs = ["spin_primitive.h"],
    deps = [
        ":primitive",
        "//firmware/app/control",
        "//firmware/app/control:bangbang",
        "//firmware/shared:physics",
    ],
)

cc_library(
    name = "stop",
    srcs = ["stop_primitive.c"],
    hdrs = ["stop_primitive.h"],
    deps = [
        ":primitive",
    ],
)
