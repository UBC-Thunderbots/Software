cc_library(
    name = "protobuf_primitive_visitor",
    srcs = ["protobuf_primitive_visitor.cpp"],
    hdrs = ["protobuf_primitive_visitor.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//shared:constants",
        "//shared:firmware_primitive_type",
        "//shared/proto:tbots_cc_proto",
        "//software/primitive:all_primitives",
        "//software/primitive:primitive_visitor",
    ],
)

cc_library(
    name = "protobuf_message_translator",
    srcs = ["protobuf_message_translator.cpp"],
    hdrs = ["protobuf_message_translator.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//shared/proto:tbots_cc_proto",
        "//software/new_geom:angle",
        "//software/new_geom:point",
        "//software/new_geom:vector",
        "//software/primitive:all_primitives",
        "//software/primitive:primitive_visitor",
        "//software/proto/message_translation:protobuf_primitive_visitor",
        "//software/world",
    ],
)
