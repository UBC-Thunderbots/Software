package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pass",
    srcs = ["pass.cpp"],
    hdrs = ["pass.h"],
    deps = [
        "//software/ai/evaluation:calc_best_shot",
        "//software/world:robot",
    ],
)

cc_test(
    name = "pass_test",
    srcs = ["pass_test.cpp"],
    deps = [
        ":pass",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "ball",
    srcs = ["ball.cpp"],
    hdrs = ["ball.h"],
    deps = [
        ":shot",
        "//shared:constants",
        "//software/world",
    ],
)

cc_test(
    name = "ball_test",
    srcs = ["ball_test.cpp"],
    deps = [
        ":ball",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "calc_best_shot",
    srcs = ["calc_best_shot.cpp"],
    hdrs = ["calc_best_shot.h"],
    deps = [
        ":shot",
        "//shared:constants",
        "//software/geom",
        "//software/new_geom:segment",
        "//software/world",
    ],
)

cc_test(
    name = "calc_best_shot_test",
    srcs = ["calc_best_shot_test.cpp"],
    deps = [
        ":calc_best_shot",
        ":shot",
        "//shared:constants",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "deflect_off_enemy_target",
    srcs = ["deflect_off_enemy_target.cpp"],
    hdrs = ["deflect_off_enemy_target.h"],
    deps = [
        ":shot",
        "//shared:constants",
        "//software/geom",
        "//software/new_geom:triangle",
        "//software/world",
    ],
)

cc_test(
    name = "deflect_off_enemy_target_test",
    srcs = ["deflect_off_enemy_target_test.cpp"],
    deps = [
        ":deflect_off_enemy_target",
        ":shot",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "detect_threat",
    srcs = ["detect_threat.cpp"],
    hdrs = ["detect_threat.h"],
    deps = [
        "//shared:constants",
        "//software/new_geom/util:intersection",
        "//software/world:ball",
        "//software/world:field",
    ],
)

cc_test(
    name = "detect_threat_test",
    srcs = ["detect_threat_test.cpp"],
    deps = [
        ":detect_threat",
        ":shot",
        "//software/test_util",
        "//software/time:timestamp",
        "//software/world:ball",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "enemy_threat",
    srcs = ["enemy_threat.cpp"],
    hdrs = ["enemy_threat.h"],
    deps = [
        ":calc_best_shot",
        ":intercept",
        ":possession",
        ":robot",
        ":shot",
        ":team",
        "//shared:constants",
        "//software/new_geom/util:intersects",
        "//software/world",
    ],
)

cc_test(
    name = "enemy_threat_test",
    srcs = ["enemy_threat_test.cpp"],
    deps = [
        ":enemy_threat",
        "//software/test_util",
        "//software/world:ball",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "find_open_areas",
    srcs = ["find_open_areas.cpp"],
    hdrs = ["find_open_areas.h"],
    deps = [
        ":shot",
        "//software/new_geom/util:find_open_circles",
        "//software/parameter:dynamic_parameters",
        "//software/world",
    ],
)

cc_library(
    name = "intercept",
    srcs = ["intercept.cpp"],
    hdrs = ["intercept.h"],
    deps = [
        ":shot",
        "//shared:constants",
        "//software/ai/evaluation:pass",
        "//software/optimization:gradient_descent",
        "//software/world:ball",
        "//software/world:field",
        "//software/world:robot",
    ],
)

cc_test(
    name = "intercept_test",
    srcs = ["intercept_test.cpp"],
    deps = [
        ":intercept",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "possession",
    srcs = ["possession.cpp"],
    hdrs = ["possession.h"],
    deps = [
        ":intercept",
        ":robot",
        ":shot",
        ":team",
        "//software/parameter:dynamic_parameters",
        "//software/world:ball",
        "//software/world:field",
        "//software/world:robot",
    ],
)

cc_test(
    name = "possession_test",
    srcs = ["possession_test.cpp"],
    deps = [
        ":possession",
        "//shared:constants",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "robot",
    srcs = ["robot.cpp"],
    hdrs = ["robot.h"],
    deps = [
        ":shot",
        "//shared:constants",
        "//software/parameter:dynamic_parameters",
        "//software/world",
    ],
)

cc_test(
    name = "robot_test",
    srcs = ["robot_test.cpp"],
    deps = [
        ":robot",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "team",
    srcs = ["team.cpp"],
    hdrs = ["team.h"],
    deps = [
        "//software/world:robot",
        "//software/world:team",
    ],
)

cc_test(
    name = "team_test",
    srcs = ["team_test.cpp"],
    deps = [
        ":team",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "shot",
    srcs = ["shot.cpp"],
    hdrs = ["shot.h"],
    deps = ["//software/new_geom:point"],
)
