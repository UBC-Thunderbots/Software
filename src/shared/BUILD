package(default_visibility = ["//visibility:public"])

#added platformio rules to make constants library compatible with arduino board build tool
load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
)
load("@bazel_skylib//rules:common_settings.bzl", "string_flag")

platformio_library(
    name = "constants_platformio",
    hdr = "constants.h",
)

cc_library(
    name = "constants",
    srcs = [],
    hdrs = ["constants.h"],
    alwayslink = True,
)

cc_library(
    name = "crc",
    srcs = [
        "crc.c",
    ],
    hdrs = [
        "crc.h",
    ],
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
    deps = [":constants"],
)
