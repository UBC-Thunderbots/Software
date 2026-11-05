load("@gameplay_tests_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_library", "py_test")

py_library(
    name = "conftest",
    testonly = True,
    srcs = ["conftest.py"],
    deps = [requirement("pytest")],
)

py_test(
    name = "runtime_manifest_test",
    srcs = ["runtime_manifest_test.py"],
    deps = [
        ":conftest",
        "//software/gameplay_tests:util",
        "//software/thunderscope/binary_context_managers:runtime_manifest",
        requirement("pytest"),
    ],
)
