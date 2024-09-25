package(default_visibility = ["//visibility:public"])

cc_library(
    name = "primitive",
    hdrs = [
        "primitive.h",
    ],
    deps = [
        "//proto:tbots_cc_proto",
        "//software/ai/navigator/obstacle:robot_navigation_obstacle_factory",
    ],
)

cc_library(
    name = "move_primitive",
    srcs = ["move_primitive.cpp"],
    hdrs = [
        "move_primitive.h",
    ],
    deps = [
        ":primitive",
        "//proto/message_translation:tbots_protobuf",
        "//proto/primitive:primitive_msg_factory",
        "//software/ai/navigator/trajectory:trajectory_planner",
        "//software/geom/algorithms:end_in_obstacle_sample",
    ],
)

cc_library(
    name = "stop_primitive",
    srcs = ["stop_primitive.cpp"],
    hdrs = [
        "stop_primitive.h",
    ],
    deps = [
        ":primitive",
        "//proto/message_translation:tbots_protobuf",
    ],
)

cc_test(
    name = "primitive_test",
    srcs = ["primitive_test.cpp"],
    deps = [
        ":move_primitive",
        ":stop_primitive",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
