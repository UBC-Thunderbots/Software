package(default_visibility = ["//visibility:public"])

cc_library(
    name = "get_behind_ball_tactic",
    srcs = [
        "get_behind_ball_fsm.cpp",
        "get_behind_ball_tactic.cpp",
    ],
    hdrs = [
        "get_behind_ball_fsm.h",
        "get_behind_ball_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/primitive:move_primitive",
        "//software/ai/hl/stp/tactic",
        "//software/geom:triangle",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "get_behind_ball_fsm_test",
    srcs = ["get_behind_ball_fsm_test.cpp"],
    deps = [
        ":get_behind_ball_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
