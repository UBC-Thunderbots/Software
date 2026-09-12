load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base64",
    srcs = [
        "base64.cpp",
    ],
    hdrs = [
        "base64.h",
    ],
)
