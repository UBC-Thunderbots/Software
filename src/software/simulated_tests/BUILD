package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulated_test_fixture",
    srcs = ["simulated_test_fixture.cpp"],
    hdrs = ["simulated_test_fixture.h"],
    deps = [
        "//software/ai",
        "//software/gui/visualizer:visualizer_wrapper",
        "//software/gui/visualizer/drawing:navigator",
        "//software/logger",
        "//software/sensor_fusion",
        "//software/simulated_tests/validation:continuous_function_validator",
        "//software/simulated_tests/validation:function_validator",
        "//software/simulation:simulator",
        "//software/test_util",
        "//software/time:duration",
        "@gtest",
    ],
)

cc_test(
    name = "simulated_test_fixture_test",
    srcs = ["simulated_test_fixture_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        "//software/simulated_tests/validation:validation_function",
        "//software/time:duration",
        "//software/time:timestamp",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "example_play_test",
    srcs = ["example_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        "//software/ai/hl/stp/play:example_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/time:duration",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "kickoff_friendly_play_test",
    srcs = ["kickoff_friendly_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        "//software/ai/hl/stp/play:kickoff_friendly_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/time:duration",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "kickoff_enemy_play_test",
    srcs = ["kickoff_enemy_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        "//software/ai/hl/stp/play:kickoff_enemy_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/time:duration",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)
