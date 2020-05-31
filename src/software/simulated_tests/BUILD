package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mock_ai",
    srcs = ["mock_ai_wrapper.cpp"],
    hdrs = ["mock_ai_wrapper.h"],
    deps = [
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
        "//software/primitive",
        "//software/world",
    ],
)

cc_library(
    name = "simulated_test_fixture",
    srcs = ["simulated_test_fixture.cpp"],
    hdrs = ["simulated_test_fixture.h"],
    deps = [
        "//software/ai:ai_wrapper",
        "//software/backend:simulator_backend",
        "//software/gui/visualizer:visualizer_wrapper",
        "//software/logger",
        "//software/simulated_tests/validation:world_state_validator",
        "//software/time:duration",
        "@gtest",
    ],
)

cc_test(
    name = "simulated_tests_test",
    srcs = ["simulated_tests_test.cpp"],
    deps = [
        ":mock_ai",
        ":simulated_test_fixture",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
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
        "//software/test_util",
        "//software/time:duration",
        "//software/time:timestamp",
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
        "//software/test_util",
        "//software/time:duration",
        "//software/time:timestamp",
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
        "//software/test_util",
        "//software/time:duration",
        "//software/time:timestamp",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)
