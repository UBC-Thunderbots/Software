package(default_visibility = ["//visibility:public"])

cc_library(
    name = "calc_best_shot",
    srcs = ["calc_best_shot.cpp"],
    hdrs = ["calc_best_shot.h"],
    deps = [
        "//shared:constants",
        "//software/ai/world",
    ],
)

cc_library(
    name = "deflect_off_enemy_target",
    srcs = ["deflect_off_enemy_target.cpp"],
    hdrs = ["deflect_off_enemy_target.h"],
    deps = [
        "//shared:constants",
        "//software/ai/world",
        "//software/geom:all",
    ],
)

cc_library(
    name = "detect_threat",
    srcs = ["detect_threat.cpp"],
    hdrs = ["detect_threat.h"],
    deps = [
        "//shared:constants",
        "//software/ai/world:ball",
        "//software/ai/world:field",
        "//software/geom:all",
    ],
)

cc_library(
    name = "enemy_threat",
    srcs = ["enemy_threat.cpp"],
    hdrs = ["enemy_threat.h"],
    deps = [
        "//shared:constants",
        "//software/ai/world",
        "//software/geom:all",
    ],
)

cc_library(
    name = "find_open_areas",
    srcs = ["find_open_areas.cpp"],
    hdrs = ["find_open_areas.h"],
    deps = [
        ":indirect_chip",
        "//software/ai/world",
        "//software/geom:all",
        "//software/util/parameter:dynamic_parameters",
    ],
)

cc_library(
    name = "indirect_chip",
    srcs = ["indirect_chip.cpp"],
    hdrs = ["indirect_chip.h"],
    deps = [
        "//shared:constants",
        "//software/ai/world",
        "//software/geom:all",
        "//software/util/parameter:dynamic_parameters",
    ],
)
