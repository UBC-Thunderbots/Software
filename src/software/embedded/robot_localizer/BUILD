load("@bazel_skylib//rules:common_settings.bzl", "bool_flag", "string_flag")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_shell//shell:sh_binary.bzl", "sh_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "robot_localizer",
    srcs = ["robot_localizer.cpp"],
    hdrs = ["robot_localizer.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//proto/primitive:primitive_msg_factory",
        "//software:constants",
        "//software/embedded/services:imu",
        "//software/geom:angle",
        "//software/geom:angular_velocity",
        "//software/geom:point",
        "//software/geom:vector",
        "//software/physics:velocity_conversion_util",
        "//software/sensor_fusion/filter:extended_kalman_filter",
        "//software/sensor_fusion/filter:kalman_filter",
        "//software/world:robot_state",
        "@eigen",
    ],
)

cc_test(
    name = "robot_localizer_test",
    srcs = ["robot_localizer_test.cpp"],
    deps = [
        ":robot_localizer",
        "//shared:constants",
        "//shared/test_util:tbots_gtest_main",
        "//software/physics:velocity_conversion_util",
    ],
)
