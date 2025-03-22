package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "shadow_enemy_tactic",
    srcs = [
        "shadow_enemy_fsm.cpp",
        "shadow_enemy_tactic.cpp",
    ],
    hdrs = [
        "shadow_enemy_fsm.h",
        "shadow_enemy_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "shadow_enemy_fsm_test",
    srcs = ["shadow_enemy_fsm_test.cpp"],
    deps = [
        ":shadow_enemy_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
