package(default_visibility = ["//visibility:public"])

cc_library(
    name = "detection",
    srcs = ["detection.cpp"],
    hdrs = ["detection.h"],
    deps = [
        "//software/geom",
        "//software/util/time:timestamp",
    ],
)

cc_library(
    name = "refbox_data",
    srcs = ["refbox_data.cpp"],
    hdrs = ["refbox_data.h"],
    deps = [
        "//software/geom",
        "//software/util/time:timestamp",
        "//software/proto:ssl_cc_proto",
        "@magic_enum",
    ],
)

