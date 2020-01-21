package(default_visibility = ["//visibility:public"])

# TODO (Issue #790): remove all `strip_include_prefix` once firmware has been setup to have includes
#       from top-level

cc_library(
    name = "control",
    srcs = ["control.c"],
    hdrs = ["control.h"],
    deps = [
        "//firmware/main/app/world:firmware_robot",
        "//firmware/main/physics",
        "//firmware/main/shared:util",
    ],
)
