load("@gameplay_tests_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_test")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "field_frame_test",
    srcs = ["field_frame_test.py"],
    deps = [
        "//software/gameplay_tests:util",
        "//software/thunderscope:field_frame",
        requirement("pytest"),
    ],
)
