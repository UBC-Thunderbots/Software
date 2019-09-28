package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ball",
    srcs = ["ball.cpp"],
    hdrs = ["ball.h"],
    deps = [
        "//shared:constants",
        "//software/ai/world:ball",
        "//software/geom",
        "//software/gui/geom:geometry_conversion",
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "field",
    srcs = ["field.cpp"],
    hdrs = ["field.h"],
    deps = [
        "//shared:constants",
        "//software/ai/world:field",
        "//software/gui/geom:geometry_conversion",
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "robot",
    srcs = ["robot.cpp"],
    hdrs = ["robot.h"],
    deps = [
        "//shared:constants",
        "//software/ai/world:robot",
        "//software/geom",
        "//software/gui/geom:geometry_conversion",
        "@qt//:qt_gui",
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "team",
    srcs = ["team.cpp"],
    hdrs = ["team.h"],
    deps = [
        ":robot",
        "//software/ai/world:team",
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "world",
    srcs = ["world.cpp"],
    hdrs = ["world.h"],
    deps = [
        ":ball",
        ":draw_functions",
        ":field",
        ":team",
        "//software/ai/world",
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "navigator",
    srcs = ["navigator.cpp"],
    hdrs = ["navigator.h"],
    deps = [
        ":draw_functions",
        "//software/ai/navigator",
        "//software/geom",
        "//software/gui/geom:geometry_conversion",
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "draw_functions",
    srcs = [],
    hdrs = ["draw_functions.h"],
    deps = [
        "@qt//:qt_widgets",
    ],
)
