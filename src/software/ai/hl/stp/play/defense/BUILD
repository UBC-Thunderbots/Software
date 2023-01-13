package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

# We force linking for all plays so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "defense_play",
    srcs = [
        "defense_play.cpp",
        "defense_play_fsm.cpp",
    ],
    hdrs = [
        "defense_play.h",
        "defense_play_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:defense_lane",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/ai/hl/stp/tactic/pass_defender:pass_defender_tactic",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/util/generic_factory",
        "@sml",
    ],
    alwayslink = True,
)

py_test(
    name = "defense_play_test",
    srcs = [
        "defense_play_test.py",
    ],
    # TODO (#2619) Remove tag to run in parallel
    tags = [
        "exclusive",
    ],
    deps = [
        "//software/simulated_tests:simulated_test_fixture",
        "//software/simulated_tests:validation",
        requirement("pytest"),
    ],
)

cc_test(
    name = "defense_play_fsm_test",
    srcs = ["defense_play_fsm_test.cpp"],
    deps = [
        ":defense_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
