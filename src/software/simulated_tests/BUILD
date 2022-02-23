load("@pybind11_bazel//:build_defs.bzl", "pybind_extension", "pybind_library")
load("@com_google_protobuf//:protobuf.bzl", "py_proto_library")
load("@simulated_tests_deps//:requirements.bzl", "requirement")

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

cc_library(
    name = "tactic_stepper",
    srcs = ["tactic_stepper.cpp"],
    hdrs = ["tactic_stepper.h"],
    deps = [
        "//software/ai/hl/stp/tactic",
        "//software/ai/navigator",
        "//software/ai/navigator/path_manager:velocity_obstacle_path_manager",
        "//software/ai/navigator/path_planner:theta_star_path_planner",
    ],
)

PYBIND_DEPS = [
    "@local_config_python//:python_headers",
    "@pybind11",
]

cc_library(
    name = "pyton_bindings_lib",
    srcs = ["python_bindings.cpp"],
    copts = ["-fexceptions"],
    features = [
        "-use_header_modules",
        "-parse_headers",
    ],
    deps = [
        "@pybind11_protobuf//pybind11_protobuf:native_proto_caster",
        "//software/ai:ai",
        "//software/ai/hl/stp/tactic/attacker:attacker_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/simulated_tests:tactic_stepper",
        "//software/sensor_fusion",
        "//shared/parameter:cpp_configs",
        "//proto:sensor_msg_cc_proto",
    ] + PYBIND_DEPS,
)

pybind_extension(
    name = "python_bindings",
    deps = [
        ":pyton_bindings_lib",
    ],
)

py_test(
    name = "simulated_test_fixture",  # TODO nuke the py
    srcs = [
        "simulated_test_fixture.py",
    ],
    data = [
        ":python_bindings.so",
    ],
    deps = [
        ":standalone_simulator_wrapper",
        "//proto:software_py_proto",
        "//software/networking:threaded_unix_listener_py",
        "//software/networking:threaded_unix_sender_py",
        "//software/thunderscope",
        "//software/thunderscope:colors",
        "//software/thunderscope:constants",
        "//software/thunderscope/field",
        "//software/thunderscope/field:field_layer",
        "//software/thunderscope/field:obstacle_layer",
        "//software/thunderscope/field:world_layer",
        "//software/thunderscope/log:g3log_widget",
        requirement("pytest"),
    ],
)

py_test(
    name = "standalone_simulator_wrapper",
    srcs = [
        "standalone_simulator_wrapper.py",
    ],
    data = [
        "//software/simulation:standalone_er_force_simulator_main"
    ],
    deps = [
        "//proto:software_py_proto",
        "//software/networking:threaded_unix_listener_py",
        "//software/networking:threaded_unix_sender_py",
    ],
)
