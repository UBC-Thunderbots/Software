package(default_visibility = ["//visibility:public"])

# We force linking for all plays so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "shoot_or_pass_play",
    srcs = [
        "shoot_or_pass_play.cpp",
        "shoot_or_pass_play_fsm.cpp",
    ],
    hdrs = [
        "shoot_or_pass_play.h",
        "shoot_or_pass_play_fsm.h",
    ],
    deps = [
        "//proto/message_translation:tbots_protobuf",
        "//shared:constants",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/attacker:attacker_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/receiver:receiver_tactic",
        "//software/ai/passing:eighteen_zone_pitch_division",
        "//software/ai/passing:pass_generator",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/util/generic_factory",
        "@sml",
        "@tracy",
    ],
    alwayslink = True,
)

cc_test(
    name = "shoot_or_pass_play_fsm_test",
    srcs = ["shoot_or_pass_play_fsm_test.cpp"],
    deps = [
        ":shoot_or_pass_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "shoot_or_pass_play_test",
    srcs = ["shoot_or_pass_play_test.cpp"],
    deps = [
        ":shoot_or_pass_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
