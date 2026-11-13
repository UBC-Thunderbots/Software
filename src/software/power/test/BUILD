load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "power_board_runtime_tests",
    srcs = ["power_board_runtime_tests.cpp"],
    deps = [
        "//proto/message_translation:power_frame_msg",
        "//shared/test_util:tbots_gtest_main",
        "//shared/uart_framing",
        "//software/power:power_board_runtime_cc",
        "//software/power:sim_hal",
    ],
)
