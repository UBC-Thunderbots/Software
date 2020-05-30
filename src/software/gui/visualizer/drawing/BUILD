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
    name = "geom",
    srcs = ["geom.cpp"],
    hdrs = ["geom.h"],
    deps = [
        "//software/geom",
        "//software/gui/visualizer/geom:geometry_conversion",
        "@qt//:qt_core",
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
        ":geom",
        "//shared:constants",
        "//software/geom",
        "//software/gui/visualizer/geom:geometry_conversion",
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
        ":geom",
        "//shared:constants",
        "//software/gui/visualizer/geom:geometry_conversion",
        "//software/world:field",
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "robot",
    srcs = ["robot.cpp"],
    hdrs = ["robot.h"],
    deps = [
        ":geom",
        "//shared:constants",
        "//software/geom",
        "//software/gui/visualizer/geom:geometry_conversion",
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
    name = "obstacle_artist",
    srcs = ["obstacle_artist.cpp"],
    hdrs = ["obstacle_artist.h"],
    deps = [
        ":colors",
        ":draw_functions",
        ":geom",
        "//software/ai/navigator/obstacle",
        "//software/ai/navigator/obstacle:obstacle_visitor",
        "//software/geom",
        "//software/gui/visualizer/geom:geometry_conversion",
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
        ":geom",
        ":obstacle_artist",
        "//software/ai/navigator",
        "//software/geom",
        "//software/gui/visualizer/geom:geometry_conversion",
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
