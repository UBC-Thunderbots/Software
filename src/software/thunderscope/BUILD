package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_binary(
    name = "thunderscope",
    srcs = ["thunderscope.py"],
    deps = [
        "//proto:software_py_proto",
        "//software/networking:threaded_unix_listener_py",
        "//software/thunderscope:colors",
        "//software/thunderscope:common_widgets",
        "//software/thunderscope:constants",
        "//software/thunderscope/chicker",
        "//software/thunderscope/field",
        "//software/thunderscope/field:field_layer",
        "//software/thunderscope/log:g3log_widget",
        requirement("pyqtgraph"),
        requirement("numpy"),
    ],
)

py_library(
    name = "constants",
    srcs = ["constants.py"],
)

py_library(
    name = "colors",
    srcs = ["colors.py"],
)

py_library(
    name = "common_widgets",
    srcs = ["common_widgets.py"],
)
