load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("@gameplay_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

compile_pip_requirements(
    name = "requirements",
    src = "requirements.in",
    requirements_txt = "requirements_lock.txt",
)

py_library(
    name = "fixture",
    srcs = [
        "fixture.py",
    ],
    deps = [
        ":util",
        ":field_test_runner",
        ":simulated_test_runner",
        "//software/thunderscope:estop_helpers",
        "//software/logger:py_logger",
        "//software/thunderscope:thunderscope",
        "//software/thunderscope/binary_context_managers:full_system",
        "//software/thunderscope/binary_context_managers:game_controller",
        "//software/thunderscope/binary_context_managers:simulator",
        requirement("pytest"),
    ],
)

py_library(
    name = "tbots_test_runner",
    srcs = [
        "tbots_test_runner.py",
    ],
    deps = [
        "//proto:import_all_protos",
        "//software/logger:py_logger",
        "//software/networking/unix:threaded_unix_listener_py",
        "//software/networking/unix:threaded_unix_sender_py",
        "//software/gameplay_tests/validation:validations",
        "//software/thunderscope:thunderscope",
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
    deps = [],
)

py_library(
    name = "simulated_test_runner",
    srcs = [
        "simulated_test_runner.py",
    ],
    deps = [],
)

py_library(
    name = "util",
    srcs = [
        "util.py",
    ],
    deps = [],
)
