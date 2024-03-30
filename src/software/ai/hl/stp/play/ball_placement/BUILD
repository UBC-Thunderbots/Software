package(default_visibility = ["//visibility:public"])

# We force linking for all plays so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "ball_placement_play",
    srcs = [
        "ball_placement_play.cpp",
        "ball_placement_play_fsm.cpp",
    ],
    hdrs = [
        "ball_placement_play.h",
        "ball_placement_play_fsm.h",
    ],
    deps = [
        "//proto/message_translation:tbots_protobuf",
        "//shared:constants",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/assigned_skill:assigned_skill_tactics",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/passing:eighteen_zone_pitch_division",
        "//software/logger",
        "//software/util/generic_factory",
        "@sml",
    ],
    alwayslink = True,
)

py_test(
    name = "ball_placement_play_test",
    srcs = [
        "ball_placement_play_test.py",
    ],
    tags = ["exclusive"],
    deps = [
        "//software/simulated_tests:simulated_test_fixture",
        "//software/simulated_tests:validation",
        requirement("pytest"),
    ],
)

cc_test(
    name = "ball_placement_play_fsm_test",
    srcs = ["ball_placement_play_fsm_test.cpp"],
    deps = [
        ":ball_placement_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
