package(default_visibility = ["//visibility:public"])

py_library(
    name = "proto_configuration_widget",
    srcs = ["proto_configuration_widget.py"],
    deps = [
        "//software/thunderscope/common:proto_parameter_tree_util",
    ],
)

py_library(
    name = "proto_plotter",
    srcs = ["proto_plotter.py"],
    deps = [
        "//software/thunderscope:thread_safe_buffer",
    ],
)

py_library(
    name = "common_widgets",
    srcs = ["common_widgets.py"],
)

py_library(
    name = "proto_parameter_tree_util",
    srcs = ["proto_parameter_tree_util.py"],
    deps = [
        ":common_widgets",
    ],
)

py_library(
    name = "frametime_counter",
    srcs = ["frametime_counter.py"],
)

py_library(
    name = "fps_widget",
    srcs = ["fps_widget.py"],
    deps = [
        ":frametime_counter",
    ],
)
