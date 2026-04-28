package(default_visibility = ["//visibility:public"])

cc_library(
    name = "motor_index",
    hdrs = ["motor_index.h"],
    deps = [
        "//software/util/make_enum",
    ],
)

cc_library(
    name = "motor_board",
    hdrs = ["motor_board.h"],
    deps = [
        "//software/util/make_enum",
    ],
)

cc_library(
    name = "motor_controller",
    srcs = [
        "stspin_motor_controller.cpp",
        "tmc_motor_controller.cpp",
    ],
    hdrs = [
        "motor_controller.h",
        "stspin_motor_controller.h",
        "tmc_motor_controller.h",
    ],
    deps = [
        ":motor_board",
        ":motor_fault_indicator",
        ":motor_index",
        ":stspin_types",
        "//proto/message_translation:tbots_protobuf",
        "//software/embedded:spi_utils",
        "//software/embedded/gpio",
        "//software/logger",
        "//software/physics:euclidean_to_wheel",
        "@cppcrc",
        "@trinamic",
    ],
)

cc_library(
    name = "motor_fault_indicator",
    srcs = ["motor_fault_indicator.cpp"],
    hdrs = ["motor_fault_indicator.h"],
    deps = [
        ":motor_board",
        "//proto:tbots_cc_proto",
    ],
)

cc_library(
    name = "stspin_types",
    hdrs = ["stspin_types.h"],
)

cc_binary(
    name = "stspin_motor_controller_test",
    srcs = ["stspin_motor_controller_test.cpp"],
    linkopts = [
        "-lpthread",
        "-lrt",
    ],
    deps = [
        ":motor_controller",
        "//software/logger",
        "//software/logger:network_logger",
        "@boost//:program_options",
    ],
)
