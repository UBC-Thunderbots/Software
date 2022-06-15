package(default_visibility = ["//visibility:public"])

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
        "//software/simulated_tests:validation",
        "//software/thunderscope",
        "//software/thunderscope:binary_context_managers",
        "//software/thunderscope/replay:proto_logger",
    ],
)
