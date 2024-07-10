package(default_visibility = ["//visibility:public"])

#added platformio rules to make constants library compatible with arduino board build tool
load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
)
load("@bazel_skylib//rules:common_settings.bzl", "string_flag")

config_setting(
    name = "build_nano",
    flag_values = {
        ":platform": "NANO",
    },
)

config_setting(
    name = "build_pi",
    flag_values = {
        ":platform": "PI",
    },
)

string_flag(
    name = "platform",
    build_setting_default = "",
    values = [
        "PI",
        "NANO",
    ],
)

cc_library(
    name = "constants",
    srcs = [],
    hdrs = ["constants.h"],
    deps = ["//software/util/make_enum"],
    alwayslink = True,
)

platformio_library(
    name = "constants_platformio",
    hdr = "constants.h",
)

cc_library(
    name = "robot_constants",
    srcs = [
        "2021_robot_constants.cpp",
    ],
    hdrs = [
        "2021_robot_constants.h",
        "robot_constants.h",
    ],
    deps = [
        ":constants",
    ],
)

cc_test(
    name = "testington",
    srcs = ["test.cpp"],
    deps = [":thunderloop_constants"],
)

cc_library(
    name = "thunderloop_constants",
    hdrs = ["thunderloop_constants.h"],
    defines = select({
        ":build_nano": ["NANO"],
        ":build_pi": ["PI"],
        "//conditions:default": ["NONE"],
    }),
    deps = [
        ":jetson_constants",
        ":pi_constants",
    ],
)

cc_library(
    name = "jetson_constants",
    hdrs = ["jetson_constants.h"],
    deps = ["//software/util/make_enum"],
)

cc_library(
    name = "pi_constants",
    hdrs = ["pi_constants.h"],
    deps = ["//software/util/make_enum"],
)
