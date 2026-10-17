load("@gameplay_tests_deps//:requirements.bzl", "requirement")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "er_force_simulator",
    srcs = ["er_force_simulator.cpp"],
    hdrs = ["er_force_simulator.h"],
    data = [
        "//extlibs/er_force_sim/config/simulator:2020.txt",
        "//extlibs/er_force_sim/config/simulator:2020B.txt",
    ],
    deps = [
        ":simulated_robot",
        "//extlibs/er_force_sim/src/amun/simulator",
        "//proto/message_translation:ssl_detection",
        "//proto/message_translation:ssl_geometry",
        "//proto/message_translation:ssl_simulation_robot_control",
        "//proto/message_translation:ssl_wrapper",
        "//shared:constants",
        "//software/physics:velocity_conversion_util",
        "//software/world",
        "//software/world:field",
    ],
)

cc_test(
    name = "er_force_simulator_test",
    srcs = ["er_force_simulator_test.cpp"],
    deps = [
        ":er_force_simulator",
        "//proto/message_translation:er_force_world",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
        "//software/world",
    ],
)

cc_library(
    name = "simulated_imu",
    srcs = ["simulated_imu.cpp"],
    hdrs = ["simulated_imu.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//software/geom:angular_velocity",
        "//software/geom:vector",
        "//software/time:duration",
        "//software/world:robot_state",
    ],
)

cc_test(
    name = "simulated_imu_test",
    srcs = ["simulated_imu_test.cpp"],
    deps = [
        ":simulated_imu",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "simulated_motor",
    srcs = ["simulated_motor.cpp"],
    hdrs = ["simulated_motor.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//shared:robot_constants",
        "//software/physics:euclidean_to_wheel",
        "//software/time:duration",
        "//software/world:robot_state",
    ],
)

cc_test(
    name = "simulated_motor_test",
    srcs = ["simulated_motor_test.cpp"],
    deps = [
        ":simulated_motor",
        "//proto/primitive:primitive_msg_factory",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "simulated_robot",
    srcs = ["simulated_robot.cpp"],
    hdrs = ["simulated_robot.h"],
    deps = [
        ":simulated_imu",
        ":simulated_motor",
        "//proto:tbots_cc_proto",
        "//shared:robot_constants",
        "//software/embedded:primitive_executor",
        "//software/embedded/robot_localizer",
        "//software/time:duration",
        "//software/world:robot_state",
    ],
)

py_test(
    name = "simulator_ball_test",
    srcs = [
        "simulator_ball_test.py",
    ],
    deps = [
        "//software:conftest",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)
