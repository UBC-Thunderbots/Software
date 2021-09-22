package(default_visibility = ["//visibility:public"])

cc_library(
    name = "radio_output",
    srcs = ["radio_output.cpp"],
    hdrs = ["radio_output.h"],
    deps = [
        ":robot_status",
        "//software/backend/radio/mrf:dongle",
        "//software/world:ball",
        "//software/world:team",
    ],
)

cc_test(
    name = "robot_status_test",
    srcs = ["robot_status_test.cpp"],
    deps = [
        ":robot_status",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "robot_status",
    srcs = ["robot_status.cpp"],
    hdrs = ["robot_status.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//software/backend/radio/mrf:messages",
        "//software/logger",
    ],
)
