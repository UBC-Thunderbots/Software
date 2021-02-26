package(default_visibility = ["//visibility:public"])

cc_library(
    name = "primitive",
    srcs = ["primitive.c"],
    hdrs = ["primitive.h"],
    deps = [
        "//firmware/app/world:firmware_world",
        "//shared/proto:tbots_nanopb_proto",
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
        ":move",
        ":primitive",
        ":chip",
        ":autochip_move",
        ":autokick_move",
        ":direct_control",
        ":kick",
        ":spinning_move",
        ":stop",
        "//firmware/app/world:firmware_world",
    ] + select({
        # TODO: this should not be dependent on freertos at all
        "//cc_toolchain:cpu_stm32f4": ["//firmware/boards/legacy_robot_stm32f4:legacy_shared_libs"],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "direct_control",
    srcs = ["direct_control_primitive.c"],
    hdrs = ["direct_control_primitive.h"],
    deps = [
        ":primitive",
        "//firmware/app/control",
        "//firmware/app/logger",
    ],
)

cc_library(
    name = "chick_motion",
    srcs = ["chick_motion.c"],
    hdrs = ["chick_motion.h"],
    deps = [
        ":primitive",
        "//firmware/app/control",
        "//firmware/app/control:bangbang",
        "//firmware/app/control:physbot",
        "//firmware/app/control:trajectory_planner",
        "//firmware/shared:physics",
        "//firmware/shared:util",
    ],
)

cc_library(
    name = "chip",
    srcs = [
        "chip_primitive.c",
    ],
    hdrs = ["chip_primitive.h"],
    deps = [
        "chick_motion",
        ":primitive",
    ],
)

cc_library(
    name = "kick",
    srcs = ["kick_primitive.c"],
    hdrs = ["kick_primitive.h"],
    deps = [
        "chick_motion",
        ":primitive",
    ],
)

cc_library(
    name = "spinning_move",
    srcs = ["spinning_move_primitive.c"],
    hdrs = ["spinning_move_primitive.h"],
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
        "//firmware/app/control:trajectory_planner",
        "//firmware/shared:physics",
        "//firmware/shared:util",
        "//shared:constants",
        "//shared:robot_constants",
    ],
)

cc_library(
    name = "autokick_move",
    srcs = ["autokick_move_primitive.c"],
    hdrs = ["autokick_move_primitive.h"],
    deps = [
        "move_helper",
        ":primitive",
    ],
)

cc_library(
    name = "autochip_move",
    srcs = ["autochip_move_primitive.c"],
    hdrs = ["autochip_move_primitive.h"],
    deps = [
        "move_helper",
        ":primitive",
    ],
)

cc_test(
    name = "autochip_move_test",
    srcs = ["autochip_move_primitive_test.cpp"],
    deps = [
        ":autochip_move",
        ":test_util_world",
    ],
)

cc_library(
    name = "test_util_world",
    hdrs = ["test_util_world.h"],
    deps = [
        ":primitive_manager",
        "@fff",
        "@gtest//:gtest_main",
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
