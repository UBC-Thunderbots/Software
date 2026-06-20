load("@bazel_skylib//rules:expand_template.bzl", "expand_template")
load("//shared:practice_field_dims.bzl", "practice_field_substitutions")

package(default_visibility = ["//visibility:public"])

exports_files([
    "2020.txt",
    "2020B.txt",
])

expand_template(
    name = "practice_txt",
    out = "practice.txt",
    substitutions = practice_field_substitutions(),
    template = "practice.txt.tmpl",
)
