package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bangbang",
    srcs = ["bangbang.c"],
    hdrs = ["bangbang.h"],
    deps = [],
)

cc_library(
    name = "control",
    srcs = ["control.c"],
    hdrs = ["control.h"],
    deps = [
        "//firmware/app/world:firmware_robot",
        "//firmware/shared:physics",
        "//firmware/shared:util",
    ],
)

cc_library(
    name = "physbot",
    srcs = ["physbot.c"],
    hdrs = ["physbot.h"],
    deps = [
        ":bangbang",
        "//firmware/app/world:firmware_robot",
        "//firmware/shared:physics",
    ],
)
