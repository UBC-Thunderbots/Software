package(default_visibility = ["//visibility:private"])

cc_library(
    name = "simulated_action_test_fixture",
    testonly = True,
    srcs = ["simulated_action_test_fixture.cpp"],
    hdrs = ["simulated_action_test_fixture.h"],
    deps = [
        "//software/ai/hl/stp/action",
        "//software/ai/navigator",
        "//software/ai/navigator/path_manager:velocity_obstacle_path_manager",
        "//software/ai/navigator/path_planner:theta_star_path_planner",
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
    name = "simulated_action_test_main",
    testonly = True,
    srcs = ["simulated_action_test_main.cpp"],
    deps = [
        ":simulated_action_test_fixture",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
        "@gtest",
    ],
)

cc_test(
    name = "move_action_test",
    srcs = ["move_action_test.cpp"],
    deps = [
        ":simulated_action_test_fixture",
        ":simulated_action_test_main",
        "//software/ai/hl/stp/action:move_action",
        "//software/simulated_tests/validation:validation_function",
        "//software/simulated_tests/validation_functions",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
