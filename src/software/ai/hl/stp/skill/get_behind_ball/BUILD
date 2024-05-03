package(default_visibility = ["//visibility:public"])

# We force linking for all Skills so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "get_behind_ball_skill",
    srcs = [
        "get_behind_ball_skill_fsm.cpp",
    ],
    hdrs = [
        "get_behind_ball_skill_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/primitive:move_primitive",
        "//software/ai/hl/stp/skill",
        "//software/ai/hl/stp/tactic:transition_conditions",
        "//software/geom:triangle",
        "//software/geom/algorithms",
    ],
    alwayslink = True,
)

cc_test(
    name = "get_behind_ball_skill_fsm_test",
    srcs = ["get_behind_ball_skill_fsm_test.cpp"],
    deps = [
        ":get_behind_ball_skill",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
