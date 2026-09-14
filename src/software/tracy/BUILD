load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_shell//shell:sh_binary.bzl", "sh_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tracy_constants",
    hdrs = ["tracy_constants.h"],
)

sh_binary(
    name = "tracy",
    srcs = ["tracy.sh"],
)
