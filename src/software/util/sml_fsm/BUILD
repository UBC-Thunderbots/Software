load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sml_fsm",
    srcs = ["sml_fsm.cpp"],
    hdrs = ["sml_fsm.h"],
    deps = [
        "//software/util/typename",
        "@sml",
    ],
)
