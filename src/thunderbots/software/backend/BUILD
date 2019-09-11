package(default_visibility = ["//visibility:public"])

cc_library(
    name = "backend",
    hdrs = ["backend.h"],
    deps = [
        ":robot_status",
        "//software:typedefs",
        "//software/ai/primitive",
        "//software/ai/world",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
    ],
)

cc_library(
    name = "backend_factory",
    srcs = ["backend_factory.cpp"],
    hdrs = ["backend_factory.h"],
    deps = [":backend"],
)

cc_library(
    name = "grsim_backend",
    srcs = ["grsim_backend.cpp"],
    hdrs = ["grsim_backend.h"],
    deps = [
        ":backend",
        ":backend_factory",
        "//software/backend/input/network/networking:network_client",
        "//software/backend/output/grsim:grsim_output",
    ],
)

cc_library(
    name = "radio_backend",
    srcs = ["radio_backend.cpp"],
    hdrs = ["radio_backend.h"],
    deps = [
        ":backend",
        ":backend_factory",
        "//software/backend/input/network/networking:network_client",
        "//software/backend/output/radio:radio_output",
    ],
)

cc_library(
    name = "robot_status",
    hdrs = ["robot_status.h"],
    deps = [],
)
