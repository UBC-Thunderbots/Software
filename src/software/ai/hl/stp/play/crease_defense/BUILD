package(default_visibility = ["//visibility:public"])

# We force linking for all plays so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "crease_defense_play",
    srcs = [
        "crease_defense_play.cpp",
        "crease_defense_play_fsm.cpp",
    ],
    hdrs = [
        "crease_defense_play.h",
        "crease_defense_play_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/util/generic_factory",
        "@sml",
    ],
    alwayslink = True,
)

cc_test(
    name = "crease_defense_play_test",
    srcs = ["crease_defense_play_test.cpp"],
    deps = [
        ":crease_defense_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
    ],
)

cc_test(
    name = "crease_defense_play_fsm_test",
    srcs = ["crease_defense_play_fsm_test.cpp"],
    deps = [
        ":crease_defense_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
