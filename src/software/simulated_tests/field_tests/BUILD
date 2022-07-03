package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

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
        "//software/logger:py_logger",
        "//software/networking:threaded_unix_listener_py",
        "//software/networking:threaded_unix_sender_py",
        "//software/simulated_tests:pytest_main",
        "//software/simulated_tests:tbots_test_runner",
        "//software/simulated_tests:validation",
        "//software/thunderscope",
        "//software/thunderscope:binary_context_managers",
        "//software/thunderscope:robot_communication",
        "//software/thunderscope/replay:proto_logger",
    ],
)

py_test(
    name = "movement_field_test",
    srcs = [
        "movement_field_test.py",
    ],
    # TODO (#2619) Remove tag to run in parallel
    tags = [
        "exclusive",
    ],
    deps = [
        "//software/simulated_tests:simulated_test_fixture",
        "//software/simulated_tests:validation",
        "//software/simulated_tests/field_tests:field_test_fixture",
        requirement("pytest"),
    ],
)
