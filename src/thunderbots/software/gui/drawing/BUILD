package(default_visibility = ["//visibility:public"])

# TODO: Break this up
cc_library(
    name = "drawing",
    srcs = [
        "ball.cpp",
        "field.cpp",
        "robot.cpp",
        "team.cpp",
        "world.cpp",
    ],
    hdrs = [
        "ball.h",
        "field.h",
        "robot.h",
        "team.h",
        "world.h",
    ],
    deps = [
        "//software/ai/world",
        "//software/ai/world:ball",
        "//software/ai/world:field",
        "//software/ai/world:robot",
        "//software/ai/world:team",
        "//software/geom:all",
        "//software/gui/geom:geometry_conversion",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
        "@qt//:qt_widgets",
    ],
)
