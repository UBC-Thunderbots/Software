load("@bazel_skylib//rules:common_settings.bzl", "string_flag")
load("@bazel_skylib//rules:expand_template.bzl", "expand_template")

#added platformio rules to make constants library compatible with arduino board build tool
load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
)
load("//shared:practice_field_dims.bzl", "practice_field_substitutions")

package(default_visibility = ["//visibility:public"])

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
    name = "robot_constants",
    srcs = [],
    hdrs = [":robot_constants.h"],
    deps = [":constants"],
)

expand_template(
    name = "practice_field_dims_h",
    out = "practice_field_dims.h",
    substitutions = practice_field_substitutions(),
    template = "practice_field_dims.h.tmpl",
)

cc_library(
    name = "practice_field_dims",
    hdrs = [":practice_field_dims_h"],
)

expand_template(
    name = "practice_field_dims_py",
    out = "practice_field_dims.py",
    substitutions = practice_field_substitutions(),
    template = "practice_field_dims.py.tmpl",
)

py_library(
    name = "practice_field_dims_py_lib",
    srcs = [":practice_field_dims_py"],
)
