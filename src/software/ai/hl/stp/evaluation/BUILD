package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ball",
    srcs = ["ball.cpp"],
    hdrs = ["ball.h"],
    deps = [
        "//shared:constants",
        "//software/ai/world",
        "//software/geom",
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
        "//shared:constants",
        "//software/ai/world",
        "//software/geom",
    ],
)

cc_test(
    name = "calc_best_shot_test",
    srcs = ["calc_best_shot_test.cpp"],
    deps = [
        ":calc_best_shot",
        "//shared:constants",
        "//software/geom",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "deflect_off_enemy_target",
    srcs = ["deflect_off_enemy_target.cpp"],
    hdrs = ["deflect_off_enemy_target.h"],
    deps = [
        "//shared:constants",
        "//software/ai/world",
        "//software/geom",
    ],
)

cc_test(
    name = "deflect_off_enemy_target_test",
    srcs = ["deflect_off_enemy_target_test.cpp"],
    deps = [
        ":deflect_off_enemy_target",
        "//software/geom",
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
        "//software/ai/world:ball",
        "//software/ai/world:field",
        "//software/geom",
    ],
)

cc_test(
    name = "detect_threat_test",
    srcs = ["detect_threat_test.cpp"],
    deps = [
        ":detect_threat",
        "//software/ai/world:ball",
        "//software/geom",
        "//software/test_util",
        "//software/util/time:timestamp",
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
        ":team",
        "//shared:constants",
        "//software/ai/world",
        "//software/geom",
    ],
)

cc_test(
    name = "enemy_threat_test",
    srcs = ["enemy_threat_test.cpp"],
    deps = [
        ":enemy_threat",
        "//software/ai/world:ball",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "find_open_areas",
    srcs = ["find_open_areas.cpp"],
    hdrs = ["find_open_areas.h"],
    deps = [
        ":indirect_chip",
        "//software/ai/world",
        "//software/geom",
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
        "//software/geom",
        "//software/util/parameter:dynamic_parameters",
    ],
)

cc_test(
    name = "indirect_chip_test",
    srcs = ["indirect_chip_test.cpp"],
    deps = [
        ":indirect_chip",
        "//software/ai/world",
        "//software/geom",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "intercept",
    srcs = ["intercept.cpp"],
    hdrs = ["intercept.h"],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:pass",
        "//software/ai/world:ball",
        "//software/ai/world:field",
        "//software/ai/world:robot",
        "//software/geom",
        "//software/util/optimization:gradient_descent",
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
        ":team",
        "//shared:constants",
        "//software/ai/world:ball",
        "//software/ai/world:field",
        "//software/ai/world:robot",
        "//software/geom",
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
        "//shared:constants",
        "//software/ai/world:ball",
        "//software/ai/world:robot",
        "//software/geom",
    ],
)

cc_test(
    name = "robot_test",
    srcs = ["robot_test.cpp"],
    deps = [
        ":robot",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "team",
    srcs = ["team.cpp"],
    hdrs = ["team.h"],
    deps = [
        "//software/ai/world:robot",
        "//software/ai/world:team",
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
