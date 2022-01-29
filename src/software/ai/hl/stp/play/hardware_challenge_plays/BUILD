package(default_visibility = ["//visibility:public"])

# We force linking for all plays so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "scoring_with_static_defenders_play",
    srcs = ["scoring_with_static_defenders_play.cpp"],
    hdrs = ["scoring_with_static_defenders_play.h"],
    deps = [
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "scoring_with_static_defenders_play_test",
    srcs = ["scoring_with_static_defenders_play_test.cpp"],
    deps = [
        ":scoring_with_static_defenders_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_library(
    name = "scoring_from_contested_possession_play",
    srcs = ["scoring_from_contested_possession_play.cpp"],
    hdrs = ["scoring_from_contested_possession_play.h"],
    deps = [
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "scoring_from_contested_possession_play_test",
    srcs = ["scoring_from_contested_possession_play_test.cpp"],
    deps = [
        ":scoring_from_contested_possession_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_library(
    name = "dribbling_parcour_play",
    srcs = ["dribbling_parcour_play.cpp"],
    hdrs = ["dribbling_parcour_play.h"],
    deps = [
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "dribbling_parcour_play_test",
    srcs = ["dribbling_parcour_play_test.cpp"],
    deps = [
        ":dribbling_parcour_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_library(
    name = "pass_endurance_play",
    srcs = ["pass_endurance_play.cpp"],
    hdrs = ["pass_endurance_play.h"],
    deps = [
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "pass_endurance_play_test",
    srcs = ["pass_endurance_play_test.cpp"],
    deps = [
        ":pass_endurance_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
