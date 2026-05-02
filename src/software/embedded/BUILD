load("@bazel_skylib//rules:common_settings.bzl", "string_flag")

package(default_visibility = ["//visibility:public"])

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
        "//software/embedded:robot_localizer",
        "//software/embedded/services:imu",
        "//software/embedded/services:motor",
        "//software/embedded/services:power",
        "//software/embedded/services/network",
        "//software/embedded/toml_config",
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

filegroup(
    name = "hash_thunderloop_binary",
    srcs = ["hash_thunderloop_binary.sh"],
)

cc_test(
    name = "test_battery",
    srcs = ["battery_test.cpp"],
    deps = [
        ":thunderloop",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "robot_localizer",
    srcs = ["robot_localizer.cpp"],
    hdrs = ["robot_localizer.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//software:constants",
        "//software/embedded/services:imu",
        "//software/geom:angle",
        "//software/geom:angular_velocity",
        "//software/geom:point",
        "//software/geom:vector",
        "//software/sensor_fusion/filter:kalman_filter",
        "//software/util/scoped_timespec_timer",
        "@eigen",
    ],
)
