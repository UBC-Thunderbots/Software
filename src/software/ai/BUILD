package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ai",
    srcs = ["ai.cpp"],
    hdrs = ["ai.h"],
    # We include all the plays here so that the static
    # variables in the .cpp files that auto-register plays with the
    # PlayFactory. This addresses the issue explained here:
    # https://www.bfilipek.com/2018/02/static-vars-static-lib.html
    deps = [
        "//software/ai/hl/stp",
        "//software/ai/hl/stp:play_info",
        "//software/ai/hl/stp/play:all_plays",
        "//software/ai/hl/stp/play:halt_play",
        "//software/ai/navigator",
        "//software/ai/navigator/path_manager:velocity_obstacle_path_manager",
        "//software/ai/navigator/path_planner:theta_star_path_planner",
        "//software/time:timestamp",
        "//software/world",
    ],
)

cc_library(
    name = "threaded_ai",
    srcs = ["threaded_ai.cpp"],
    hdrs = ["threaded_ai.h"],
    deps = [
        "//shared/parameter_v2:cpp_configs",
        "//software/ai",
        "//software/gui/drawing:draw_functions",
        "//software/gui/drawing:navigator",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
        "//software/world",
        "@boost//:bind",
    ],
)
