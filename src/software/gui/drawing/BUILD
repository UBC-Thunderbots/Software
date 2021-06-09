package(default_visibility = [
    "//software/ai:__pkg__",
    "//software/gui:__subpackages__",
    "//software/simulated_tests:__subpackages__",
])

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
        "//software/geom:circle",
        "//software/geom:polygon",
        "//software/geom:rectangle",
        "//software/geom:segment",
        "//software/gui:geometry_conversion",
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
        "//software/gui:geometry_conversion",
        "//software/math:math_functions",
        "//software/sensor_fusion/filter:vision_detection",
        "//software/world:ball_state",
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
        "//software/world:field",
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "robot",
    srcs = ["robot.cpp"],
    hdrs = ["robot.h"],
    deps = [
        ":colors",
        ":geom",
        "//shared:constants",
        "//software/geom/algorithms",
        "//software/gui:geometry_conversion",
        "//software/math:math_functions",
        "//software/sensor_fusion/filter:vision_detection",
        "//software/world:robot_state",
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
        ":geom",
        ":team",
        "//software/ai/evaluation:calc_best_shot",
        "//software/gui:geometry_conversion",
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
        "//software/gui:geometry_conversion",
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
        "//software/gui:geometry_conversion",
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "draw_functions",
    hdrs = ["draw_functions.h"],
    deps = [
        "@qt//:qt_widgets",
    ],
)

cc_library(
    name = "ssl_wrapper_packet",
    srcs = ["ssl_wrapper_packet.cpp"],
    hdrs = ["ssl_wrapper_packet.h"],
    deps = [
        ":ball",
        ":colors",
        ":draw_functions",
        ":field",
        ":robot",
        "//software/proto:ssl_cc_proto",
        "//software/proto/message_translation:ssl_detection",
        "//software/proto/message_translation:ssl_geometry",
    ],
)
