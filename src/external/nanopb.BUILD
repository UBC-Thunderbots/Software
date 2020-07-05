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

# TODO: what does this comment apply to?
# this library is linked with all c_proto_libraries
# the generated h and c files access pb.h relatively, so
# we strip external/nanopb to comply

cc_library(
    name = "nanopb_header",
    hdrs = [
        "pb.h",
    ],
    defines = common_defines,
    # TODO: for some reason if we remove this line it breaks.... figure out why and
    #       fix root cause or at least add a comment
    strip_include_prefix = "",
    visibility = ["//visibility:public"],
)

py_binary(
    name = "nanopb_generator",
    srcs = ["generator/nanopb_generator.py"],
    imports = ["proto"],
)

proto_library(
    name = "nanopb_proto",
    srcs = ["generator/proto/nanopb.proto"],
    strip_import_prefix = "generator/proto",
    deps = [
        "@com_google_protobuf//:descriptor_proto",
    ],
)
