package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sml_fsm",
    srcs = ["sml_fsm.cpp"],
    hdrs = ["sml_fsm.h"],
    deps = [
        ":fsm_logger",
        "//software/util/typename",
        "@sml",
    ],
)

cc_library(
    name = "fsm_logger",
    srcs = ["fsm_logger.cpp"],
    hdrs = ["fsm_logger.h"],
    deps = [
        "//proto:play_info_msg_cc_proto",
        "//software/logger",
        "@sml",
    ],
)
