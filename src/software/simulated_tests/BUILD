package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulated_test_fixture",
    testonly = True,
    srcs = ["simulated_test_fixture.cpp"],
    hdrs = ["simulated_test_fixture.h"],
    deps = [
        "//software/ai",
        "//software/gui/drawing:navigator",
        "//software/gui/full_system:threaded_full_system_gui",
        "//software/logger",
        "//software/proto/message_translation:primitive_google_to_nanopb_converter",
        "//software/proto/message_translation:tbots_protobuf",
        "//software/sensor_fusion",
        "//software/simulated_tests/validation:non_terminating_function_validator",
        "//software/simulated_tests/validation:terminating_function_validator",
        "//software/simulation:simulator",
        "//software/test_util",
        "//software/time:duration",
        "@gtest",
    ],
)

cc_library(
    name = "simulated_test_main",
    testonly = True,
    srcs = ["simulated_test_main.cpp"],
    deps = [
        ":simulated_test_fixture",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
        "@gtest",
    ],
)

cc_test(
    name = "simulated_test_fixture_test",
    srcs = ["simulated_test_fixture_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        ":simulated_test_main",
        "//software/simulated_tests/validation:validation_function",
        "//software/time:duration",
        "//software/time:timestamp",
        "//software/world",
    ],
)

cc_test(
    name = "example_play_test",
    srcs = ["example_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        ":simulated_test_main",
        "//software/ai/hl/stp/play:example_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "kickoff_friendly_play_test",
    srcs = ["kickoff_friendly_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        ":simulated_test_main",
        "//software/ai/hl/stp/play:kickoff_friendly_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "kickoff_enemy_play_test",
    srcs = ["kickoff_enemy_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        ":simulated_test_main",
        "//software/ai/hl/stp/play:kickoff_enemy_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "corner_kick_play_test",
    srcs = ["corner_kick_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        ":simulated_test_main",
        "//software/ai/hl/stp/play:corner_kick_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/simulated_tests/validation_functions",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "defense_play_test",
    srcs = ["defense_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        ":simulated_test_main",
        "//software/ai/hl/stp/play:defense_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "enemy_free_kick_play_test",
    srcs = ["enemy_free_kick_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        ":simulated_test_main",
        "//software/ai/hl/stp/play:enemy_free_kick_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "stop_play_test",
    srcs = ["stop_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        ":simulated_test_main",
        "//software/ai/hl/stp/play:stop_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "halt_play_test",
    srcs = ["halt_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        ":simulated_test_main",
        "//software/ai/hl/stp/play:halt_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "free_kick_play_test",
    srcs = ["free_kick_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        ":simulated_test_main",
        "//software/ai/hl/stp/play:free_kick_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "shoot_or_pass_play_test",
    srcs = ["shoot_or_pass_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        ":simulated_test_main",
        "//software/ai/hl/stp/play:shoot_or_pass_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "shoot_or_chip_play_test",
    srcs = ["shoot_or_chip_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        ":simulated_test_main",
        "//software/ai/hl/stp/play:shoot_or_chip_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "penalty_kick_play_test",
    srcs = ["penalty_kick_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        ":simulated_test_main",
        "//software/ai/hl/stp/play:penalty_kick_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "penalty_kick_enemy_play_test",
    srcs = ["penalty_kick_enemy_play_test.cpp"],
    deps = [
        ":simulated_test_fixture",
        ":simulated_test_main",
        "//software/ai/hl/stp/play:penalty_kick_enemy_play",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
