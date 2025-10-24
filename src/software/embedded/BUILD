load("@bazel_skylib//rules:common_settings.bzl", "string_flag")

package(default_visibility = ["//visibility:public"])

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

config_setting(
    name = "build_limited",
    flag_values = {
        ":host_platform": "LIMITED",
    },
)

string_flag(
    name = "host_platform",
    build_setting_default = "PI",
    values = [
        "PI",
        "NANO",
        "LIMITED",
    ],
)

config_setting(
    name = "build_trinamic",
    flag_values = {
        ":motor_board": "TRINAMIC",
    },
)

config_setting(
    name = "build_stspin",
    flag_values = {
        ":motor_board": "STSPIN",
    },
)

string_flag(
    name = "motor_board",
    build_setting_default = "TRINAMIC",
    values = [
        "TRINAMIC",
        "STSPIN",
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
        "//software/time:duration",
        "//software/world:robot_state",
        "//software/world:team_colour",
    ],
)

cc_library(
    name = "spi_utils",
    srcs = ["spi_utils.cpp"],
    hdrs = ["spi_utils.h"],
    deps = [
        "//software/logger",
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
        "@tracy",
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
        "@boost//:program_options",
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
