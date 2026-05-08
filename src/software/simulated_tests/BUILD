load("@aspect_rules_py//py:defs.bzl", "py_binary", "py_library", "py_test")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

compile_pip_requirements(
    name = "requirements",
    src = "requirements.in",
    requirements_txt = "requirements_lock.txt",
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
        "//software/networking/unix:threaded_unix_listener_py",
        "//software/networking/unix:threaded_unix_sender_py",
        "//software/simulated_tests/validation:validations",
        "//software/thunderscope",
        "//software/thunderscope:config",
        "//software/thunderscope:constants",
        "//software/thunderscope:time_provider",
        "//software/thunderscope/binary_context_managers:full_system",
        "//software/thunderscope/binary_context_managers:game_controller",
        "//software/thunderscope/binary_context_managers:simulator",
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
        "//software/networking/unix:threaded_unix_listener_py",
        "//software/networking/unix:threaded_unix_sender_py",
        "//software/simulated_tests/validation:validations",
        "//software/thunderscope",
        "//software/thunderscope/binary_context_managers:full_system",
        "//software/thunderscope/binary_context_managers:game_controller",
        "//software/thunderscope/binary_context_managers:simulator",
    ],
)

# TODO (#2581): remove
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
        "//software/simulated_tests/cpp_validation:non_terminating_function_validator",
        "//software/simulated_tests/cpp_validation:terminating_function_validator",
        "//software/simulation:er_force_simulator",
        "//software/test_util",
        "//software/time:duration",
        "//shared/test_util:tbots_gtest_main",
        "//shared/test_util",
        # TODO (#1889) Remove this dep after optional params are implemented
        "//software/ai/hl/stp/play/halt_play",
        "//proto/message_translation:er_force_world",
    ],
)

# TODO (#2581): remove
cc_library(
    name = "simulated_er_force_sim_play_test_fixture",
    testonly = True,
    srcs = ["simulated_er_force_sim_play_test_fixture.cpp"],
    hdrs = ["simulated_er_force_sim_play_test_fixture.h"],
    deps = [
        ":simulated_er_force_sim_test_fixture",
        "//software/ai",
        "@googletest//:gtest",
    ],
)

py_test(
    name = "simulated_test_ball_model",
    srcs = [
        "simulated_test_ball_model.py",
    ],
    package_collisions = "warning",
    # TODO (#2619) Remove tag to run in parallel
    tags = [
        "exclusive",
    ],
    deps = [
        "//software:conftest",
        "//software/simulated_tests/validation:validations",
        requirement("pytest"),
        requirement("pyqt6"),
        requirement("numpy"),
        requirement("pyqtdarktheme-fork"),
        requirement("pyqtgraph"),
    ],
)
