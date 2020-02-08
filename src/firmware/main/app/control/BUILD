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
        "//firmware/main/app/world:firmware_robot",
        "//firmware/main/shared:physics",
        "//firmware/main/shared:util",
    ],
)

cc_library(
    name = "physbot",
    srcs = ["physbot.c"],
    hdrs = ["physbot.h"],
    deps = [
        ":bangbang",
        "//firmware/main/app/world:firmware_robot",
        "//firmware/main/shared:physics",
    ],
)
