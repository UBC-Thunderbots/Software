package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulated_test_fixture_cpp",
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
        ":simulated_test_fixture_cpp",
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
        ":simulated_test_fixture_cpp",
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

py_library(
    name = "simulated_test_fixture",
    srcs = [
        "simulated_test_fixture.py",
    ],
    deps = [
        ":er_force_simulator",
        ":full_system",
        "//proto:software_py_proto",
        "//proto:tbots_py_proto",
        "//software/logger:py_logger",
        "//software/networking:threaded_unix_listener_py",
        "//software/networking:threaded_unix_sender_py",
        "//software/simulated_tests/eventually_validation",
        "//software/thunderscope",
    ],
)

py_library(
    name = "er_force_simulator",
    srcs = [
        "er_force_simulator.py",
    ],
    data = [
        "//software:er_force_simulator_main",
    ],
    deps = [
        "//proto:software_py_proto",
        "//proto:tbots_py_proto",
        "//software/logger:py_logger",
        "//software/networking:threaded_unix_listener_py",
        "//software/networking:threaded_unix_sender_py",
    ],
)

py_library(
    name = "full_system",
    srcs = [
        "full_system.py",
    ],
    data = [
        "//software:unix_full_system",
        "//software/geom:geometry.so",
    ],
    deps = [
        "//proto:software_py_proto",
        "//proto:tbots_py_proto",
        "//software/logger:py_logger",
        "//software/networking:threaded_unix_listener_py",
        "//software/networking:threaded_unix_sender_py",
    ],
)

py_library(
    name = "validation",
    srcs = [
        "validation.py",
    ],
    data = [
        "//software/geom:geometry.so",
    ],
    deps = [
        "//proto:software_py_proto",
        "//proto:tbots_py_proto",
        "//software/logger:py_logger",
    ],
)
