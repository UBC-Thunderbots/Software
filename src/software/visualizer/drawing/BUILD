package(default_visibility = ["//visibility:public"])

cc_library(
    name = "colors",
    hdrs = ["colors.h"],
    deps = [
        "@qt//:qt_gui",
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "ball",
    srcs = ["ball.cpp"],
    hdrs = ["ball.h"],
    deps = [
        ":colors",
        "//shared:constants",
        "//software/geom",
        "//software/visualizer/geom:geometry_conversion",
        "//software/world:ball",
        "//software/world:field",
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "field",
    srcs = ["field.cpp"],
    hdrs = ["field.h"],
    deps = [
        ":colors",
        "//shared:constants",
        "//software/visualizer/geom:geometry_conversion",
        "//software/world:field",
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "robot",
    srcs = ["robot.cpp"],
    hdrs = ["robot.h"],
    deps = [
        "//shared:constants",
        "//software/geom",
        "//software/visualizer/geom:geometry_conversion",
        "//software/world:robot",
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "team",
    srcs = ["team.cpp"],
    hdrs = ["team.h"],
    deps = [
        ":colors",
        ":robot",
        "//software/world:team",
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
        "//software/world",
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "navigator",
    srcs = ["navigator.cpp"],
    hdrs = ["navigator.h"],
    deps = [
        ":colors",
        ":draw_functions",
        "//software/ai/navigator",
        "//software/geom",
        "//software/visualizer/geom:geometry_conversion",
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
