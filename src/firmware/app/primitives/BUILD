package(default_visibility = ["//visibility:public"])

cc_library(
    name = "direct_velocity",
    srcs = ["direct_velocity_primitive.c"],
    hdrs = ["direct_velocity_primitive.h"],
    deps = [
        ":primitive",
        "//firmware/app/control",
        "//shared/proto:tbots_nanopb_proto",
    ],
)

cc_library(
    name = "direct_wheels",
    srcs = ["direct_wheels_primitive.c"],
    hdrs = ["direct_wheels_primitive.h"],
    deps = [
        ":primitive",
        "//shared/proto:tbots_nanopb_proto",
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
        "//shared/proto:tbots_nanopb_proto",
    ],
)

cc_library(
    name = "primitive",
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
        ":direct_velocity",
        ":direct_wheels",
        ":move",
        ":primitive",
        ":shoot",
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
        "//shared/proto:tbots_nanopb_proto",
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
        "//shared/proto:tbots_nanopb_proto",
    ],
)

cc_library(
    name = "move_helper",
    srcs = ["move_helper.c"],
    hdrs = ["move_helper.h"],
    deps = [
        ":primitive",
        "//firmware/app/control",
        "//firmware/app/control:bangbang",
        "//firmware/app/control:physbot",
        "//firmware/shared:physics",
        "//firmware/shared:util",
        "//shared:constants",
        "//shared:robot_constants",
        "//shared/proto:tbots_nanopb_proto",
    ],
)

cc_library(
    name = "move_new",
    srcs = ["move_new_primitive.c"],
    hdrs = ["move_new_primitive.h"],
    deps = [
        "move_helper",
        ":primitive",
        "//shared/proto:tbots_nanopb_proto",
    ],
)

cc_library(
    name = "autokick_move",
    srcs = ["autokick_move_primitive.c"],
    hdrs = ["autokick_move_primitive.h"],
    deps = [
        "move_helper",
        ":primitive",
        "//shared/proto:tbots_nanopb_proto",
    ],
)

cc_library(
    name = "autochip_move",
    srcs = ["autochip_move_primitive.c"],
    hdrs = ["autochip_move_primitive.h"],
    deps = [
        "move_helper",
        ":primitive",
        "//shared/proto:tbots_nanopb_proto",
    ],
)

cc_library(
    name = "stop",
    srcs = ["stop_primitive.c"],
    hdrs = ["stop_primitive.h"],
    deps = [
        ":primitive",
        "//shared/proto:tbots_nanopb_proto",
    ],
)
