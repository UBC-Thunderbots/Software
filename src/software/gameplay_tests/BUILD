load("@gameplay_tests_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library", "py_test")
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
    name = "field_test_runner",
    srcs = [
        "field_test_runner.py",
    ],
    data = [
        "//software:py_constants.so",
    ],
    deps = [
        ":tbots_test_runner",
        "//proto:import_all_protos",
        "//software/logger:py_logger",
        "//software/thunderscope",
        "//software/thunderscope/binary_context_managers:full_system",
        "//software/thunderscope/binary_context_managers:game_controller",
        "//software/thunderscope/binary_context_managers:simulator",
        requirement("pytest"),
    ],
)

py_library(
    name = "simulated_test_runner",
    srcs = [
        "simulated_test_runner.py",
    ],
    data = [
        "//software:py_constants.so",
    ],
    deps = [
        ":tbots_test_runner",
        "//proto:import_all_protos",
        "//software/logger:py_logger",
        "//software/thunderscope",
        requirement("pytest"),
    ],
)

py_library(
    name = "fixture",
    srcs = [
        "fixture.py",
    ],
    deps = [
        ":field_test_runner",
        ":simulated_test_runner",
        ":util",
        "//software/logger:py_logger",
        "//software/thunderscope",
        "//software/thunderscope:estop_helpers",
        "//software/thunderscope:robot_communication",
        "//software/thunderscope/binary_context_managers:full_system",
        "//software/thunderscope/binary_context_managers:game_controller",
        "//software/thunderscope/binary_context_managers:simulator",
        requirement("pytest"),
    ],
)

py_library(
    name = "util",
    srcs = [
        "util.py",
    ],
    deps = [
        "//software/thunderscope:qt_dependency_bootstrap",
        requirement("pytest"),
    ],
)

py_library(
    name = "csv_logger",
    srcs = [
        "csv_logger.py",
    ],
)

py_library(
    name = "plotjuggler_sender",
    srcs = [
        "plotjuggler_sender.py",
    ],
    data = [
        "//software:py_constants.so",
    ],
)

py_library(
    name = "open_loop_field_test_runner",
    srcs = [
        "open_loop_field_test_runner.py",
    ],
    data = [
        "//software:py_constants.so",
        "//software:python_bindings.so",
    ],
    deps = [
        ":csv_logger",
        ":plotjuggler_sender",
        ":util",
        "//proto:import_all_protos",
        "//software/logger:py_logger",
        "//software/thunderscope",
        "//software/thunderscope:config",
        "//software/thunderscope:constants",
        "//software/thunderscope:estop_helpers",
        "//software/thunderscope:robot_communication",
        "//software/thunderscope:thread_safe_buffer",
        "//software/thunderscope:wifi_communication_manager",
        requirement("pytest"),
    ],
)

py_library(
    name = "open_loop_field_test_helpers",
    srcs = [
        "open_loop_field_test_helpers.py",
    ],
    deps = [
        ":open_loop_field_test_runner",
        "//software/logger:py_logger",
    ],
)

py_test(
    name = "open_loop_field_square_test",
    srcs = [
        "open_loop_field_square_test.py",
    ],
    deps = [
        ":open_loop_field_test_helpers",
        "//software:conftest",
        requirement("pytest"),
    ],
)

py_test(
    name = "open_loop_field_angle_test",
    srcs = [
        "open_loop_field_angle_test.py",
    ],
    deps = [
        ":open_loop_field_test_helpers",
        "//software:conftest",
        requirement("pytest"),
    ],
)
