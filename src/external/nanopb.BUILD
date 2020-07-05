licenses(["notice"])

exports_files(["LICENSE.txt"])

package(default_visibility = ["//visibility:public"])

common_defines = [
    "PB_FIELD_16BIT",
]

cc_library(
    name = "nanopb",
    srcs = [
        "pb_common.c",
        "pb_decode.c",
        "pb_encode.c",
    ],
    hdrs = [
        "pb.h",
        "pb_common.h",
        "pb_decode.h",
        "pb_encode.h",
    ],
    defines = common_defines,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "nanopb_header",
    hdrs = [
        "pb.h",
    ],
    defines = common_defines,
    # NOTE: Even an empty `strip_include_prefix` is prepended with the external project
    # path (ex. "external/nanopb") so that other targets can include headers by a path
    # relative to this project root. Note that if `strip_include_prefix` is not specified
    # *at all*, then all other libraries may *only* access headers from this lib via the
    # full project path (ex. `external/nanopb/pb.h`).
    strip_include_prefix = "",
    visibility = ["//visibility:public"],
)

py_binary(
    name = "nanopb_generator",
    srcs = ["generator/nanopb_generator.py"],
    imports = ["proto"],
)

proto_library(
    name = "nanopb_options_proto",
    srcs = ["generator/nanopb/options.proto"],
    strip_import_prefix = "generator/",
    deps = [
        "@com_google_protobuf//:descriptor_proto",
    ],
)
