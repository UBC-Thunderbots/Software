package(default_visibility = ["//software/ai/navigator/path_planning_navigator:__subpackages__"])

cc_library(
    name = "path_planner",
    hdrs = ["path_planner.h"],
    deps = [
        "//software/ai/navigator/path_planning_navigator/obstacle",
        "//software/geom:all",
    ],
)

cc_library(
    name = "straight_line_path_planner",
    srcs = ["straight_line_path_planner.cpp"],
    hdrs = ["straight_line_path_planner.h"],
    deps = [":path_planner"],
)

cc_library(
    name = "theta_star_path_planner",
    srcs = ["theta_star_path_planner.cpp"],
    hdrs = ["theta_star_path_planner.h"],
    deps = [
        ":path_planner",
        "//software/ai/world:ball",
        "//software/ai/world:field",
    ],
)
