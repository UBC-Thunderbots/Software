package(default_visibility = ["//visibility:public"])

# We force linking for all Skills so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "chip_skill",
    srcs = [
        "chip_skill_fsm.cpp",
    ],
    hdrs = [
        "chip_skill_fsm.h",
    ],
    deps = [
        "//software/ai/hl/stp/primitive:move_primitive",
        "//software/ai/hl/stp/skill",
        "//software/ai/hl/stp/skill/get_behind_ball:get_behind_ball_skill",
    ],
    alwayslink = True,
)

cc_test(
    name = "chip_skill_fsm_test",
    srcs = ["chip_skill_fsm_test.cpp"],
    deps = [
        ":chip_skill",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "chip_skill_test",
    srcs = ["chip_skill_test.cpp"],
    deps = [
        ":chip_skill",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
    ],
)
