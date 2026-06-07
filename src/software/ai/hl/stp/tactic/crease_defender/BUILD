load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "crease_defender_tactic",
    srcs = [
        "crease_defender_fsm.cpp",
        "crease_defender_tactic.cpp",
    ],
    hdrs = [
        "crease_defender_fsm.h",
        "crease_defender_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/defender:defender_fsm_base",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/geom:stadium",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "crease_defender_fsm_test",
    srcs = ["crease_defender_fsm_test.cpp"],
    deps = [
        ":crease_defender_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

py_test(
    name = "crease_defender_tactic_test",
    srcs = ["crease_defender_tactic_test.py"],
    tags = [
        "exclusive",
    ],
    deps = [
        "//software:conftest",
        "//software/simulated_tests/validation:validations",
        requirement("pytest"),
    ],
)
