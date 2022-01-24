package(default_visibility = ["//visibility:public"])

cc_library(
    name = "primitive_executor",
    srcs = ["primitive_executor.cpp"],
    hdrs = ["primitive_executor.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//proto/primitive:primitive_msg_factory",
        "//shared:constants",
        "//software/math:math_functions",
        "//software/world",
    ],
)

cc_library(
    name = "thunderloop",
    srcs = ["thunderloop.cpp"],
    hdrs = ["thunderloop.h"],
    deps = [
        ":primitive_executor",
        "//proto:tbots_cc_proto",
        "//software/jetson_nano/services:motor",
    ],
)

cc_library(
    name = "gpio",
    srcs = ["gpio.cpp"],
    hdrs = ["gpio.h"],
    deps = [
        "//software/logger",
        "//software/util/make_enum",
    ],
)

cc_binary(
    name = "thunderloop_main",
    srcs = ["thunderloop_main.cpp"],
    linkopts = [
        "-lpthread",
        "-lrt",
    ],
    deps = [
        ":thunderloop",
        "//shared:constants",
    ],
)

cc_binary(
    name = "motor_interface",
    srcs = ["motor_interface.c"],
    copts = ["-pthread"],
    linkopts = [
        "-pthread",
        "-lrt",
    ],
    deps = [
        "@trinamic",
    ],
)
