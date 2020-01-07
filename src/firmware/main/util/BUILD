package(default_visibility = ["//visibility:public"])

# TODO: remove all `strip_include_prefix` once firmware has been setup to have includes
#       from top-level

cc_library(
    name = "util",
    srcs = glob(["*.c"]),
    hdrs = glob(["*.h"]),
    strip_include_prefix = "/firmware/main",
)
