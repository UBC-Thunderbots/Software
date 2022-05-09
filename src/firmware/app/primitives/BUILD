package(default_visibility = ["//visibility:public"])

cc_library(
    name = "primitive",
    srcs = ["primitive.c"],
    hdrs = ["primitive.h"],
    deps = [
        "//firmware/app/world:firmware_world",
        "//proto:tbots_nanopb_proto",
    ],
)

cc_library(
    name = "primitive_manager",
    srcs = ["primitive_manager.c"],
    hdrs = ["primitive_manager.h"],
    copts = select({
        "//conditions:default": [],
    }),
    deps = [
        ":move",
        ":primitive",
        ":stop",
        "//firmware/app/world:firmware_world",
    ] + select({
        "//conditions:default": [],
    }),
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

cc_test(
    name = "move_test",
    srcs = ["move_primitive_test.cpp"],
    deps = [
        ":move",
        ":test_util_world",
    ],
)

cc_library(
    name = "test_util_world",
    testonly = True,
    hdrs = ["test_util_world.h"],
    deps = [
        ":primitive_manager",
        "//shared/test_util",
        "//shared/test_util:tbots_gtest_main",
        "@fff",
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
