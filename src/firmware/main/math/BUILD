package(default_visibility = ["//visibility:public"])

# TODO: remove all `strip_include_prefix` once firmware has been setup to have includes
#       from top-level

cc_library(
    name = "tbots_math",
    hdrs = ["tbots_math.h"],
    strip_include_prefix = "/firmware/main",
)

cc_library(
    name = "matrix",
    hdrs = ["matrix.h"],
    strip_include_prefix = "/firmware/main",
)
