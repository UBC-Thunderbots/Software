package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sml_fsm",
    srcs = ["sml_fsm.cpp"],
    hdrs = ["sml_fsm.h"],
    deps = [
        "//software/util/typename",
        ":fsm_logger",
        "@sml",
    ],
)

cc_library(
    name = "fsm_logger",
    srcs = ["fsm_logger.cpp"],
    hdrs = ["fsm_logger.h"],
    deps = [
        "//software/logger",
        "//proto:play_info_msg_cc_proto",
        "@sml",
    ]
)