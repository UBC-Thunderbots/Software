load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "stats_result",
    srcs = ["stats_result.py"],
    data = [
        "//software:python_bindings.so",
    ],
    deps = [
        ":result_interface",
        "//proto:import_all_protos",
        "//software/evaluation/logs:event_log",
        "//software/evaluation/logs:log_interface",
    ],
)

py_test(
    name = "stats_result_test",
    srcs = [
        "stats_result_test.py",
    ],
    deps = [
        requirement("pytest"),
        ":stats_result"
    ],
)

py_library(
    name = "result_interface",
    srcs = ["result_interface.py"],
    deps = [
        "//software/evaluation/logs:log_interface",
    ],
)
