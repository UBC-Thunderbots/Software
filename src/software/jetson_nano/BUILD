package(default_visibility = ["//visibility:public"])

cc_library(
    name = "primitive_executor",
    srcs = ["primitive_executor.cpp"],
    hdrs = ["primitive_executor.h"],
    deps = [
        "//proto/primitive:primitive_msg_factory",
        "//software/world",
        "//shared:constants",
        "//firmware/app/control:trajectory_planner",
        "//proto:tbots_cc_proto",
    ],
)

cc_binary(
    name = "thunderloop",
    srcs = ["thunderloop.cpp"],
    deps = [
        "//software/jetson_nano/services:motor",
        "//proto:tbots_cc_proto",
        ":primitive_executor"
    ],
)

