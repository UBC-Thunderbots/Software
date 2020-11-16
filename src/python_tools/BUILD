package(default_visibility = ["//visibility:public"])

load("@python_tools_deps//:requirements.bzl", "requirement")

py_test(
    name = "test_hermetic_python_environment",
    srcs = ["test_hermetic_python_environment.py"],
)

py_binary(
    name = "jupyter",
    srcs = ["jupyter.py"],
    # glob all the notebooks in this directory
    data = [":notebooks"],
    deps = [
        requirement("jupyter"),
        requirement("notebook"),
        requirement("numpy"),
        requirement("matplotlib"),
        "//shared/proto:tbots_py_proto",
        "//software/proto:software_py_proto",
    ],
)
