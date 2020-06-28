cc_test(
    name = "generated_c_test",
    srcs = ["generated_c_test.cpp"],
    deps = [
        ":example_config",
        ":parameter",
        "//software/proto/message_translation:tbots_protobuf",
        "//software/test_util",
    ],
)

cc_library(
    name = "parameter",
    srcs = ["parameter.c"],
    hdrs = ["parameter.h"],
)

cc_library(
    name = "example_config",
    srcs = ["example_config.c"],
    hdrs = ["example_config.h"],
    deps = [":parameter"],
)
