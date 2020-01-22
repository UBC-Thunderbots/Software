package(default_visibility = ["//visibility:public"])

cc_library(
    name = "physics",
    srcs = ["physics.c"],
    hdrs = ["physics.h"],
    deps = [
        "//firmware/main/math:tbots_math",
        "//firmware/main/shared:util",
    ],
)

cc_library(
    name = "util",
    srcs = ["util.c"],
    hdrs = ["util.h"],
    deps = [
        "//firmware/main/math:tbots_math",
    ],
)
