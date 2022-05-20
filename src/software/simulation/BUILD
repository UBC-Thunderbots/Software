package(default_visibility = ["//visibility:public"])

cc_library(
<<<<<<< HEAD
    name = "simulator_robot",
    srcs = ["simulator_robot.cpp"],
    hdrs = ["simulator_robot.h"],
)

cc_library(
    name = "force_wheel_simulator_robot",
    hdrs = ["force_wheel_simulator_robot.h"],
    deps = [
        ":simulator_robot",
    ],
)

cc_library(
    name = "physics_simulator_robot",
    srcs = ["physics_simulator_robot.cpp"],
    hdrs = ["physics_simulator_robot.h"],
    deps = [
        ":force_wheel_simulator_robot",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/math:math_functions",
        "//software/simulation/physics:physics_ball",
        "//software/simulation/physics:physics_robot",
    ],
)

cc_test(
    name = "physics_simulator_robot_test",
    srcs = ["physics_simulator_robot_test.cpp"],
    deps = [
        ":physics_simulator_ball",
        ":physics_simulator_robot",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulation/physics:physics_world",
        "//software/test_util",
        "//software/world",
        "//software/world:robot",
        "@box2d",
    ],
)

cc_library(
    name = "simulator_ball",
    hdrs = ["simulator_ball.h"],
    deps = [
        "//software/geom:point",
        "//software/geom:vector",
    ],
)

cc_library(
    name = "physics_simulator_ball",
    srcs = ["physics_simulator_ball.cpp"],
    hdrs = ["physics_simulator_ball.h"],
    deps = [
        ":simulator_ball",
        "//software/logger",
        "//software/simulation/physics:physics_ball",
    ],
)

cc_test(
    name = "physics_simulator_ball_test",
    srcs = ["physics_simulator_ball_test.cpp"],
    deps = [
        ":physics_simulator_ball",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulation/physics:physics_ball",
        "//software/simulation/physics:physics_world",
        "//software/test_util",
        "@box2d",
    ],
)

cc_library(
    name = "simulator_robot_singleton",
    srcs = ["simulator_robot_singleton.cpp"],
    hdrs = ["simulator_robot_singleton.h"],
    deps = [
        ":simulator_robot",
        "//proto:tbots_cc_proto",
        "//software/logger",
        "//software/world:field",
    ],
)

cc_library(
    name = "force_wheel_simulator_robot_singleton",
    srcs = ["force_wheel_simulator_robot_singleton.cpp"],
    hdrs = ["force_wheel_simulator_robot_singleton.h"],
    deps = [
        ":force_wheel_simulator_robot",
        ":simulator_robot_singleton",
    ],
)

cc_test(
    name = "force_wheel_simulator_robot_singleton_test",
    srcs = ["force_wheel_simulator_robot_singleton_test.cpp"],
    deps = [
        ":force_wheel_simulator_robot_singleton",
        ":physics_simulator_ball",
        ":physics_simulator_robot",
        ":simulator_ball",
        ":simulator_robot",
        "//proto:tbots_cc_proto",
        "//proto:tbots_nanopb_proto",
        "//shared/test_util",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulation/physics:physics_robot",
        "//software/simulation/physics:physics_world",
        "//software/test_util",
        "//software/world",
        "//software/world:robot",
    ],
)

cc_library(
    name = "simulator_ball_singleton",
    srcs = ["simulator_ball_singleton.cpp"],
    hdrs = ["simulator_ball_singleton.h"],
    deps = [
        ":simulator_ball",
        "//software/logger",
        "//software/world:field",
        "@box2d",
    ],
)

cc_test(
    name = "simulator_ball_singleton_test",
    srcs = ["simulator_ball_singleton_test.cpp"],
    deps = [
        ":physics_simulator_ball",
        ":simulator_ball",
        ":simulator_ball_singleton",
        "//shared/test_util:tbots_gtest_main",
        "//software/geom:point",
        "//software/geom:vector",
        "//software/simulation/physics:physics_world",
        "//software/time:timestamp",
        "//software/world",
    ],
)

cc_library(
    name = "simulator",
    srcs = ["simulator.cpp"],
    hdrs = ["simulator.h"],
    deps = [
        ":force_wheel_simulator_robot_singleton",
        ":physics_simulator_ball",
        ":physics_simulator_robot",
        ":simulator_ball_singleton",
        "//proto:defending_side_msg_cc_proto",
        "//proto/message_translation:primitive_google_to_nanopb_converter",
        "//proto/message_translation:ssl_detection",
        "//proto/message_translation:ssl_geometry",
        "//proto/message_translation:ssl_wrapper",
        "//shared/parameter:cpp_configs",
        "//software/simulation/physics:physics_world",
        "//software/world",
        "//software/world:field",
        "//software/world:team_colour",
    ],
)

cc_library(
    name = "er_force_simulator",
    srcs = ["er_force_simulator.cpp"],
    hdrs = ["er_force_simulator.h"],
    data = [
        "//extlibs/er_force_sim/config/simulator:2020.txt",
        "//extlibs/er_force_sim/config/simulator:2020B.txt",
    ],
    deps = [
        "//extlibs/er_force_sim/src/amun/simulator:simulator_qt",
        "//proto:defending_side_msg_cc_proto",
        "//proto/message_translation:ssl_detection",
        "//proto/message_translation:ssl_geometry",
        "//proto/message_translation:ssl_simulation_robot_control",
        "//proto/message_translation:ssl_wrapper",
        "//shared/parameter:cpp_configs",
        "//software/jetson_nano:primitive_executor",
        "//software/world",
        "//software/world:field",
        "//software/world:team_colour",
    ],
)

cc_test(
    name = "er_force_simulator_test",
    srcs = ["er_force_simulator_test.cpp"],
    deps = [
        ":er_force_simulator",
        "//proto/message_translation:er_force_world",
        "//proto/primitive:primitive_msg_factory",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
        "//software/world",
    ],
)
<<<<<<< HEAD

cc_test(
    name = "simulator_test",
    srcs = ["simulator_test.cpp"],
    deps = [
        ":simulator",
        "//proto/primitive:primitive_msg_factory",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
        "//software/world",
    ],
)

cc_library(
    name = "threaded_simulator",
    srcs = ["threaded_simulator.cpp"],
    hdrs = ["threaded_simulator.h"],
    deps = [
        ":simulator",
        "//proto:defending_side_msg_cc_proto",
        "//proto:ssl_cc_proto",
    ],
)

cc_test(
    name = "threaded_simulator_test",
    srcs = ["threaded_simulator_test.cpp"],
    deps = [
        ":threaded_simulator",
        "//proto/message_translation:tbots_protobuf",
        "//proto/primitive:primitive_msg_factory",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
        "//software/world",
    ],
)

cc_library(
    name = "standalone_simulator",
    srcs = ["standalone_simulator.cpp"],
    hdrs = ["standalone_simulator.h"],
    deps = [
        ":threaded_simulator",
        "//proto:defending_side_msg_cc_proto",
        "//proto:tbots_cc_proto",
        "//proto/message_translation:primitive_google_to_nanopb_converter",
        "//shared/parameter:cpp_configs",
        "//software:constants",
        "//software/logger",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "//software/world:field",
    ],
)
=======
>>>>>>> 243b01bd4aee2b86af7f995d150a251647508142
