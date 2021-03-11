package(default_visibility = ["//visibility:public"])

cc_library(
    name = "box2d_util",
    srcs = ["box2d_util.cpp"],
    hdrs = ["box2d_util.h"],
    deps = [
        "//software/geom:convex_polygon",
        "//software/geom:point",
        "//software/geom:vector",
        "@box2d",
    ],
)

cc_test(
    name = "box2d_util_test",
    srcs = ["box2d_util_test.cpp"],
    deps = [
        ":box2d_util",
        "@box2d",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "physics_object_user_data",
    hdrs = ["physics_object_user_data.h"],
    deps = [],
)

cc_test(
    name = "physics_object_user_data_test",
    srcs = ["physics_object_user_data_test.cpp"],
    deps = [
        ":physics_object_user_data",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "physics_ball",
    srcs = ["physics_ball.cpp"],
    hdrs = ["physics_ball.h"],
    deps = [
        ":box2d_util",
        ":physics_object_user_data",
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/geom:point",
        "//software/geom:vector",
        "//software/geom/algorithms",
        "//software/physics",
        "//software/time:duration",
        "//software/world:ball_state",
        "@box2d",
    ],
)

cc_test(
    name = "physics_ball_test",
    srcs = ["physics_ball_test.cpp"],
    deps = [
        ":physics_ball",
        "//software/test_util",
        "//software/world:ball",
        "@box2d",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "physics_field",
    srcs = ["physics_field.cpp"],
    hdrs = ["physics_field.h"],
    deps = [
        ":box2d_util",
        "//software/time:timestamp",
        "//software/world:field",
        "@box2d",
    ],
)

cc_test(
    name = "physics_field_test",
    srcs = ["physics_field_test.cpp"],
    deps = [
        ":physics_ball",
        ":physics_field",
        "//software/world:field",
        "@box2d",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "physics_robot_model",
    srcs = ["physics_robot_model.cpp"],
    hdrs = ["physics_robot_model.h"],
    deps = [
        ":box2d_util",
        "//shared:constants",
        "//software/geom:point",
        "//software/world:robot_state",
        "@box2d",
    ],
)

cc_library(
    name = "physics_robot",
    srcs = ["physics_robot.cpp"],
    hdrs = ["physics_robot.h"],
    deps = [
        ":box2d_util",
        ":physics_ball",
        ":physics_object_user_data",
        ":physics_robot_model",
        "//firmware/shared:physics",
        "//shared:constants",
        "//software/geom:angle",
        "//software/geom:angular_velocity",
        "//software/geom:point",
        "//software/geom:vector",
        "//software/multithreading:thread_safe_buffer",
        "//software/world:robot_state",
        "@box2d",
    ],
)

cc_test(
    name = "physics_robot_test",
    srcs = ["physics_robot_test.cpp"],
    deps = [
        ":physics_ball",
        ":physics_robot",
        "//software/test_util",
        "//software/time:duration",
        "//software/world:robot",
        "@box2d",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "physics_world",
    srcs = ["physics_world.cpp"],
    hdrs = ["physics_world.h"],
    deps = [
        ":box2d_util",
        ":simulation_contact_listener",
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/logger",
        "//software/multithreading:thread_safe_buffer",
        "//software/simulation/physics:physics_ball",
        "//software/simulation/physics:physics_field",
        "//software/simulation/physics:physics_robot",
        "//software/time:duration",
        "//software/time:timestamp",
        "//software/world",
        "//software/world:ball_state",
        "//software/world:robot_state",
        "@box2d",
    ],
)

cc_test(
    name = "physics_world_test",
    srcs = ["physics_world_test.cpp"],
    deps = [
        ":physics_world",
        "//software/test_util",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "simulation_contact_listener",
    srcs = ["simulation_contact_listener.cpp"],
    hdrs = ["simulation_contact_listener.h"],
    deps = [
        ":physics_ball",
        ":physics_object_user_data",
        ":physics_robot",
        "//software/simulation:simulator_ball",
        "//software/simulation:simulator_robot",
        "@box2d",
    ],
)

cc_test(
    name = "simulation_contact_listener_test",
    srcs = ["simulation_contact_listener_test.cpp"],
    deps = [
        ":physics_object_user_data",
        ":simulation_contact_listener",
        "//software/test_util",
        "//software/world:ball",
        "//software/world:robot",
        "@box2d",
        "@gtest//:gtest_main",
    ],
)
