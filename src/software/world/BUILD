package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ball",
    srcs = ["ball.cpp"],
    hdrs = ["ball.h"],
    deps = [
        "//shared:constants",
        "//software/geom:point",
        "//software/geom:vector",
        "//software/physics",
        "//software/time:timestamp",
        "//software/world:ball_state",
    ],
)

cc_test(
    name = "ball_test",
    srcs = ["ball_test.cpp"],
    deps = [
        ":ball",
        "//proto/message_translation:tbots_protobuf",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "ball_state",
    srcs = ["ball_state.cpp"],
    hdrs = ["ball_state.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//software/geom:point",
        "//software/geom:vector",
    ],
)

cc_test(
    name = "ball_state_test",
    srcs = ["ball_state_test.cpp"],
    deps = [
        ":ball_state",
        "//proto/message_translation:tbots_protobuf",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "field",
    srcs = ["field.cpp"],
    hdrs = ["field.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//shared:constants",
        "//software/geom:circle",
        "//software/geom:rectangle",
        "//software/geom/algorithms",
        "//software/time:timestamp",
        "//software/util/make_enum",
    ],
)

cc_test(
    name = "field_test",
    srcs = ["field_test.cpp"],
    deps = [
        ":field",
        "//proto/message_translation:tbots_protobuf",
        "//shared:constants",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "game_state",
    srcs = ["game_state.cpp"],
    hdrs = ["game_state.h"],
    deps = [
        ":ball",
        "//software/logger",
        "//software/util/make_enum",
    ],
)

cc_test(
    name = "game_state_test",
    srcs = ["game_state_test.cpp"],
    deps = [
        ":game_state",
        "//proto/message_translation:tbots_protobuf",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_library(
    name = "team_colour",
    hdrs = ["team_types.h"],
    deps = ["//software/util/make_enum"],
)

cc_library(
    name = "robot",
    srcs = ["robot.cpp"],
    hdrs = ["robot.h"],
    deps = [
        ":robot_capabilities",
        ":robot_state",
        "//proto:tbots_cc_proto",
        "//software:constants",
        "//software/ai/evaluation:time_to_travel",
        "//software/geom:polygon",
        "//software/logger",
        "//software/time:duration",
        "//software/time:timestamp",
    ],
)

cc_test(
    name = "robot_test",
    srcs = ["robot_test.cpp"],
    deps = [
        ":robot",
        "//proto/message_translation:tbots_protobuf",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "robot_state",
    srcs = ["robot_state.cpp"],
    hdrs = ["robot_state.h"],
    deps = [
        "team_colour",
        "//proto:tbots_cc_proto",
        "//shared:constants",
        "//shared:robot_constants",
        "//software/geom:angle",
        "//software/geom:angular_velocity",
        "//software/geom:point",
        "//software/geom:vector",
        "//software/time:duration",
    ],
)

cc_test(
    name = "robot_state_test",
    srcs = ["robot_state_test.cpp"],
    deps = [
        ":robot_state",
        "//proto/message_translation:tbots_protobuf",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "robot_capabilities",
    hdrs = ["robot_capabilities.h"],
    deps = ["//software/util/make_enum"],
)

cc_test(
    name = "robot_capabilities_test",
    srcs = ["robot_capabilities_test.cpp"],
    deps = [
        "robot_capabilities",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "team",
    srcs = ["team.cpp"],
    hdrs = ["team.h"],
    deps = [
        ":robot",
        "//software/logger",
    ],
)

cc_test(
    name = "team_test",
    srcs = ["team_test.cpp"],
    deps = [
        ":team",
        "//proto/message_translation:tbots_protobuf",
        "//shared/test_util:tbots_gtest_main",
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
        "@boost//:circular_buffer",
    ],
)

cc_test(
    name = "world_test",
    srcs = ["world_test.cpp"],
    deps = [
        ":world",
        "//proto/message_translation:tbots_protobuf",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
