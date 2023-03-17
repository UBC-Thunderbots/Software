package(default_visibility = ["//visibility:public"])

cc_library(
    name = "primitive_executor",
    srcs = ["primitive_executor.cpp"],
    hdrs = ["primitive_executor.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//proto/primitive:primitive_msg_factory",
        "//shared:constants",
        "//software/ai/navigator/path_planner/hrvo",
        "//software/math:math_functions",
        "//software/physics:velocity_conversion_util",
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
        "//software/jetson_nano/redis",
        "//software/jetson_nano/services:motor",
        "//software/jetson_nano/services:power",
        "//software/jetson_nano/services/network",
        "//software/logger:network_logger",
        "//software/util/scoped_timespec_timer",
        "//software/world:team",
    ],
)

cc_library(
    name = "gpio",
    srcs = ["gpio.cpp"],
    hdrs = ["gpio.h"],
    deps = [
        "//software/logger",
        "//software/logger:network_logger",
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

sh_binary(
    name = "setup_nano",
    srcs = ["setup_nano.sh"],
)
