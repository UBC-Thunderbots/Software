package(default_visibility = ["//visibility:public"])

# TODO: remove all `strip_include_prefix` once firmware has been setup to have includes
#       from top-level

cc_library(
    name = "util",
    srcs = ["util.c"],
    hdrs = ["util.h"],
    strip_include_prefix = "/firmware/main",
    deps = [
        "//firmware/main/math:tbots_math",
    ],
)
