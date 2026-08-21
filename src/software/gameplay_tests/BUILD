load("@gameplay_tests_deps//:requirements.bzl", "requirement")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")

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
        "//software/gameplay_tests/validation:validations",
        "//software/logger:py_logger",
        "//software/networking/unix:threaded_unix_listener_py",
        "//software/networking/unix:threaded_unix_sender_py",
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
        "//software/gameplay_tests/validation:validations",
        "//software/logger:py_logger",
        "//software/networking/unix:threaded_unix_listener_py",
        "//software/networking/unix:threaded_unix_sender_py",
        "//software/thunderscope",
        "//software/thunderscope/binary_context_managers:full_system",
        "//software/thunderscope/binary_context_managers:game_controller",
        "//software/thunderscope/binary_context_managers:simulator",
    ],
)

py_library(
    name = "field_test_fixture",
    srcs = [
        "field_test_fixture.py",
    ],
    data = [
        "//software:py_constants.so",
    ],
    deps = [
        "//proto:import_all_protos",
        "//software/gameplay_tests:tbots_test_runner",
        "//software/gameplay_tests/validation:validations",
        "//software/logger:py_logger",
        "//software/networking/unix:threaded_unix_listener_py",
        "//software/networking/unix:threaded_unix_sender_py",
        "//software/thunderscope",
        "//software/thunderscope:constants",
        "//software/thunderscope:estop_helpers",
        "//software/thunderscope:robot_communication",
        "//software/thunderscope/binary_context_managers:full_system",
        "//software/thunderscope/binary_context_managers:game_controller",
        "//software/thunderscope/binary_context_managers:simulator",
    ],
)

py_test(
    name = "simulated_test_ball_model",
    srcs = [
        "simulated_test_ball_model.py",
    ],
    deps = [
        "//software:conftest",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)

py_test(
    name = "movement_robot_field_test",
    srcs = [
        "movement_robot_field_test.py",
    ],
    deps = [
        "//software:conftest",
        "//software/gameplay_tests:tbots_test_runner",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)

py_test(
    name = "pivot_kick_field_test",
    srcs = [
        "pivot_kick_field_test.py",
    ],
    deps = [
        "//software:conftest",
        "//software/gameplay_tests:tbots_test_runner",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)

py_test(
    name = "passing_field_test",
    srcs = [
        "passing_field_test.py",
    ],
    deps = [
        "//software:conftest",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)
