package(default_visibility = ["//visibility:public"])

load("@python_tools_deps//:requirements.bzl", "requirement")

py_binary(
    name = "hrvo_visualizer",
    srcs = ["hrvo_visualizer.py"],
    deps = [
        requirement("matplotlib"),
        requirement("pandas"),
    ],
)
