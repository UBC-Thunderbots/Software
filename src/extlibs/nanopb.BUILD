licenses(["notice"])

exports_files(["LICENSE.txt"])

package(default_visibility = ["//visibility:public"])

load("@nanopb_deps//:requirements.bzl", "requirement")
load("@com_google_protobuf//:protobuf.bzl", "py_proto_library")
load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
)

common_defines = [
    # By default, NanoPb only supports 8-bit tags. This define changes the tag type to
    # one that supports 16-bit tags.
    # (https://jpa.kapsi.fi/nanopb/docs/reference.html)
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
    # NOTE: Even an empty `strip_include_prefix` is prepended with the external project
    # path (ex. "external/nanopb") so that other targets can include headers by a path
    # relative to this project root. Note that if `strip_include_prefix` is not specified
    # *at all*, then all other libraries may *only* access headers from this lib via the
    # full project path (ex. `external/nanopb/pb.h`).
    strip_include_prefix = "",
    visibility = ["//visibility:public"],
)

platformio_library(
    name = "nanopb_platformio",
    src = "pb_common.c",
    add_hdrs = [
        "pb.h",
        "pb_common.h",
        "pb_decode.h",
        "pb_encode.h",
    ],
    add_srcs = [
        "pb_common.c",
        "pb_decode.c",
        "pb_encode.c",
    ],
    hdr = "pb_common.h",
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
    deps = [
        requirement("protobuf"),
    ],
)

proto_library(
    name = "nanopb_options_proto",
    srcs = ["generator/nanopb/options.proto"],
    deps = [
        "@com_google_protobuf//:descriptor_proto",
    ],
)

py_proto_library(
    name = "nanopb_options_py_proto",
    srcs = ["generator/nanopb/options.proto"],
    deps = [
        "@com_google_protobuf//:protobuf_python",
    ],
)
