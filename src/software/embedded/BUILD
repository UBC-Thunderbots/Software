package(default_visibility = ["//visibility:public"])

load("@bazel_skylib//rules:common_settings.bzl", "string_flag")

config_setting(
    name = "build_nano",
    flag_values = {
        ":host_platform": "NANO",
    },
)

config_setting(
    name = "build_pi",
    flag_values = {
        ":host_platform": "PI",
    },
)

string_flag(
    name = "host_platform",
    build_setting_default = "PI",
    values = [
        "PI",
        "NANO",
    ],
)

cc_library(
    name = "platform",
    hdrs = ["platform.h"],
    deps = ["//software/util/make_enum"],
)

cc_library(
    name = "primitive_executor",
    srcs = ["primitive_executor.cpp"],
    hdrs = ["primitive_executor.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//proto/primitive:primitive_msg_factory",
        "//software/ai/navigator/trajectory:bang_bang_trajectory_1d_angular",
        "//software/ai/navigator/trajectory:trajectory_path",
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
        "//software/embedded/redis",
        "//software/embedded/services:motor",
        "//software/embedded/services:power",
        "//software/embedded/services/network",
        "//software/logger:network_logger",
        "//software/tracy:tracy_constants",
        "//software/util/scoped_timespec_timer",
        "//software/world:team",
        "@boost//:format",
        "@tracy",
    ],
)

cc_library(
    name = "gpio",
    srcs = [
        "gpio_char_dev.cpp",
        "gpio_sysfs.cpp",
    ],
    hdrs = [
        "gpio.h",
        "gpio_char_dev.h",
        "gpio_sysfs.h",
    ],
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
        "@boost//:program_options"
    ],
)

sh_binary(
    name = "setup_robot_software_deps",
    srcs = ["setup_robot_software_deps.sh"],
)

cc_test(
    name = "test_battery",
    srcs = ["battery_test.cpp"],
    deps = [
        ":thunderloop",
        "//shared/test_util:tbots_gtest_main",
    ],
)
