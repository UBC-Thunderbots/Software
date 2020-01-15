package(default_visibility = ["//visibility:public"])

# TODO (Issue #790): remove all `strip_include_prefix` once firmware has been setup to have includes
#       from top-level

cc_library(
    name = "physics",
    srcs = ["physics.c"],
    hdrs = ["physics.h"],
    strip_include_prefix = "/firmware/main",
    deps = [
        "//firmware/main/math:tbots_math",
        "//firmware/main/shared:util",
    ],
)
