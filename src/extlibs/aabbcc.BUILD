# Description:
#   This library provides an implementation of axis-aligned
#   bounding boxes (AABBs) tree used for fast collision detection.
#   https://github.com/lohedges/aabbcc

cc_library(
    name = "aabbcc",
    srcs = glob(["src/AABB.cc"]),
    hdrs = glob(["src/AABB.h"]),
    include_prefix = "aabbcc",
    includes = [
        "./src",
    ],
    strip_include_prefix = "src/",
    visibility = ["//visibility:public"],
)
