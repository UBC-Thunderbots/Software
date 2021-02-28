package(default_visibility = ["//visibility:public"])

cc_library(
    name = "halt_test_play",
    srcs = ["halt_test_play.cpp"],
    hdrs = ["halt_test_play.h"],
    deps = [
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/test_tactics:stop_test_tactic",
        "//shared/parameter_v2:cpp_configs",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)

cc_library(
    name = "move_test_play",
    srcs = ["move_test_play.cpp"],
    hdrs = ["move_test_play.h"],
    deps = [
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/test_tactics:move_test_tactic",
        "//shared/parameter_v2:cpp_configs",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)
