load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "shoot_or_chip_play",
    srcs = [
        "shoot_or_chip_play.cpp",
        "shoot_or_chip_play_fsm.cpp",
    ],
    hdrs = [
        "shoot_or_chip_play.h",
        "shoot_or_chip_play_fsm.h",
    ],
    deps = [
        "//proto/message_translation:tbots_protobuf",
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/evaluation:find_open_areas",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/attacker:attacker_tactic",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/ai/hl/stp/tactic/halt:halt_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/shadow_enemy:shadow_enemy_tactic",
        "//software/logger",
        "//software/util/generic_factory",
        "//software/world:game_state",
    ],
    alwayslink = True,
)

cc_test(
    name = "shoot_or_chip_play_cpp_test",
    srcs = ["shoot_or_chip_play_test.cpp"],
    deps = [
        ":shoot_or_chip_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

py_test(
    name = "shoot_or_chip_play_test",
    srcs = [
        "shoot_or_chip_play_test.py",
    ],
    # TODO (#2619) Remove tag to run in parallel
    tags = [
        "exclusive",
    ],
    deps = [
        "//software:conftest",
        "//software/simulated_tests:validation",
        requirement("pytest"),
    ],
)
