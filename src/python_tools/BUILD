package(default_visibility = ["//visibility:public"])

load("@python_tools_deps//:requirements.bzl", "requirement")

py_test(
    name = "test_hermetic_python_environment",
    srcs = ["test_hermetic_python_environment.py"],
)

py_binary(
    name = "jupyter",
    srcs = ["jupyter.py"],
    deps = [
        requirement("jupyter"),
        requirement("notebook"),
    ],
)
