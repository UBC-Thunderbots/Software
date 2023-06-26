package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

py_test(
    name = "simulated_test_ball_model",
    srcs = [
        "simulated_test_ball_model.py",
    ],
    # TODO (#2619) Remove tag to run in parallel
    tags = [
        "exclusive",
    ],
    deps = [
        "//software/simulated_tests:simulated_test_fixture",
        "//software/simulated_tests:speed_threshold_helpers",
        "//software/simulated_tests:validation",
        requirement("pytest"),
    ],
)

cc_library(
    name = "simulated_er_force_sim_test_fixture",
    testonly = True,
    srcs = ["simulated_er_force_sim_test_fixture.cpp"],
    hdrs = ["simulated_er_force_sim_test_fixture.h"],
    deps = [
        "//software/logger",
        "//proto/message_translation:tbots_protobuf",
        "//proto:play_info_msg_cc_proto",
        "//software/sensor_fusion",
        "//software/simulated_tests/validation:non_terminating_function_validator",
        "//software/simulated_tests/validation:terminating_function_validator",
        "//software/simulation:er_force_simulator",
        "//software/test_util",
        "//software/time:duration",
        "//shared/test_util:tbots_gtest_main",
        "//shared/test_util",
        # TODO (#1889) Remove this dep after optional params are implemented
        "//software/ai/hl/stp/play:halt_play",
        "//proto/message_translation:er_force_world",
    ],
)

cc_library(
    name = "simulated_er_force_sim_play_test_fixture",
    testonly = True,
    srcs = ["simulated_er_force_sim_play_test_fixture.cpp"],
    hdrs = ["simulated_er_force_sim_play_test_fixture.h"],
    deps = [
        ":simulated_er_force_sim_test_fixture",
        "//software/ai",
        "@gtest",
    ],
)

py_library(
    name = "simulated_test_fixture",
    srcs = [
        "simulated_test_fixture.py",
    ],
    data = [
        "//software:py_constants.so",
    ],
    deps = [
        ":tbots_test_runner",
        "//proto:import_all_protos",
        "//software/logger:py_logger",
        "//software/networking:threaded_unix_listener_py",
        "//software/networking:threaded_unix_sender_py",
        "//software/simulated_tests:validation",
        "//software/thunderscope",
        "//software/thunderscope:binary_context_managers",
        "//software/thunderscope/replay:proto_logger",
    ],
)

py_library(
    name = "tbots_test_runner",
    srcs = [
        "tbots_test_runner.py",
    ],
    data = [
        "//software:py_constants.so",
    ],
    deps = [
        "//proto:import_all_protos",
        "//software/logger:py_logger",
        "//software/networking:threaded_unix_listener_py",
        "//software/networking:threaded_unix_sender_py",
        "//software/simulated_tests:validation",
        "//software/thunderscope",
        "//software/thunderscope:binary_context_managers",
        "//software/thunderscope:config",
        "//software/thunderscope:constants",
        "//software/thunderscope/replay:proto_logger",
    ],
)

py_library(
    name = "validation",
    srcs = [
        "ball_enters_region.py",
        "ball_moves_forward.py",
        "ball_speed_threshold.py",
        "ball_stops_in_region.py",
        "excessive_dribbling.py",
        "friendly_goal_scored.py",
        "friendly_has_ball_possession.py",
        "friendly_team_scored.py",
        "robot_enters_region.py",
        "robot_halt.py",
        "robot_speed_threshold.py",
        "validation.py",
    ],
    data = [
        "//software:py_constants.so",
        "//software:python_bindings.so",
    ],
    deps = [
        "//proto:software_py_proto",
        "//proto:tbots_py_proto",
        "//software/logger:py_logger",
    ],
)

py_library(
    name = "speed_threshold_helpers",
    srcs = ["speed_threshold_helpers.py"],
)
