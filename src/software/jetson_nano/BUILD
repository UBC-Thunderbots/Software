package(default_visibility = ["//visibility:public"])

cc_library(
    name = "primitive_executor",
    srcs = ["primitive_executor.cpp"],
    hdrs = ["primitive_executor.h"],
    deps = [
        "//firmware/app/control:trajectory_planner",
        "//proto:tbots_cc_proto",
        "//proto/primitive:primitive_msg_factory",
        "//shared:constants",
        "//software/world",
    ],
)

cc_binary(
    name = "thunderloop",
    srcs = ["thunderloop.cpp"],
    deps = [
        ":primitive_executor",
        "//proto:tbots_cc_proto",
        "//software/jetson_nano/services:motor",
    ],
)
