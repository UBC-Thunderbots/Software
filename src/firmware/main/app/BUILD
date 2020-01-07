package(default_visibility = ["//visibility:public"])

# TODO: remove all `strip_include_prefix` once firmware has been setup to have includes
#       from top-level

cc_library(
    name = "control",
    srcs = ["control.c"],
    hdrs = ["control.h"],
    strip_include_prefix = "/firmware/main",
    deps = [
        "//firmware/main/app/world:firmware_robot",
        "//firmware/main/physics",
        "//firmware/main/util",
    ],
)
