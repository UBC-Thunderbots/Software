package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulated_test_fixture",
    testonly = True,
    srcs = ["simulated_test_fixture.cpp"],
    hdrs = ["simulated_test_fixture.h"],
    deps = [
        "//software/gui/drawing:navigator",
        "//software/gui/full_system:threaded_full_system_gui",
        "//software/logger",
        "//proto/message_translation:primitive_google_to_nanopb_converter",
        "//proto/message_translation:tbots_protobuf",
        "//software/sensor_fusion",
        "//software/simulated_tests/validation:non_terminating_function_validator",
        "//software/simulated_tests/validation:terminating_function_validator",
        "//software/simulation:simulator",
        "//software/test_util",
        "//software/time:duration",
        "//proto/logging:proto_logger",
        "//shared/test_util:tbots_gtest_main",
        # TODO (#1889) Remove this dep after optional params are implemented
        "//software/ai/hl/stp/play:halt_play",
    ],
)

cc_library(
    name = "simulated_er_force_sim_test_fixture",
    testonly = True,
    srcs = ["simulated_er_force_sim_test_fixture.cpp"],
    hdrs = ["simulated_er_force_sim_test_fixture.h"],
    deps = [
        "//software/gui/drawing:navigator",
        "//software/gui/full_system:threaded_full_system_gui",
        "//software/logger",
        "//proto/message_translation:primitive_google_to_nanopb_converter",
        "//proto/message_translation:tbots_protobuf",
        "//software/sensor_fusion",
        "//software/simulated_tests/validation:non_terminating_function_validator",
        "//software/simulated_tests/validation:terminating_function_validator",
        "//software/simulation:er_force_simulator",
        "//software/test_util",
        "//software/time:duration",
        "//proto/logging:proto_logger",
        "//shared/test_util:tbots_gtest_main",
        # TODO (#1889) Remove this dep after optional params are implemented
        "//software/ai/hl/stp/play:halt_play",
    ],
)

cc_library(
    name = "simulated_play_test_fixture",
    testonly = True,
    srcs = ["simulated_play_test_fixture.cpp"],
    hdrs = ["simulated_play_test_fixture.h"],
    deps = [
        ":simulated_test_fixture",
        "//software/ai",
        "@gtest",
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

cc_library(
    name = "simulated_tactic_test_fixture",
    testonly = True,
    srcs = ["simulated_tactic_test_fixture.cpp"],
    hdrs = ["simulated_tactic_test_fixture.h"],
    deps = [
        ":simulated_test_fixture",
        "//software/ai/hl/stp/tactic",
        "//software/ai/navigator",
        "//software/ai/navigator/path_manager:velocity_obstacle_path_manager",
        "//software/ai/navigator/path_planner:theta_star_path_planner",
        "@gtest",
    ],
)

cc_library(
    name = "simulated_er_force_sim_tactic_test_fixture",
    testonly = True,
    srcs = ["simulated_er_force_sim_tactic_test_fixture.cpp"],
    hdrs = ["simulated_er_force_sim_tactic_test_fixture.h"],
    deps = [
        ":simulated_er_force_sim_test_fixture",
        "//software/ai/hl/stp/tactic",
        "//software/ai/navigator",
        "//software/ai/navigator/path_manager:velocity_obstacle_path_manager",
        "//software/ai/navigator/path_planner:theta_star_path_planner",
        "@gtest",
    ],
)

cc_test(
    name = "simulated_play_test_fixture_test",
    srcs = ["simulated_play_test_fixture_test.cpp"],
    deps = [
        ":simulated_play_test_fixture",
        "//software/simulated_tests/validation:validation_function",
        "//software/time:duration",
        "//software/time:timestamp",
        "//software/world",
    ],
)

cc_library(
    name = "tactic_stepper",
    srcs = ["tactic_stepper.cpp"],
    hdrs = ["tactic_stepper.h"],
    deps = [
        "//software/ai/hl/stp/tactic",
        "//software/ai/navigator",
        "//software/ai/navigator/path_manager:velocity_obstacle_path_manager",
        "//software/ai/navigator/path_planner:theta_star_path_planner",
        "@gtest",
    ],
)
