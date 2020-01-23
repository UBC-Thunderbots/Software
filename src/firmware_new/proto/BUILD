load("@rules_proto//proto:defs.bzl", "proto_library", "proto_lang_toolchain")
load("@com_google_protobuf//:protobuf.bzl", "proto_gen")
load(":nanopb.bzl", "c_proto_library")

package(default_visibility = ["//visibility:public"])

# add more proto here
proto_files = [
	"control.proto",
]

# proto files
proto_library(
    name = "fw_proto",
    srcs = proto_files,
    visibility = ["//visibility:private"]
)

# TODO add ctx to macro https://github.com/UBC-Thunderbots/Software/issues/1133
c_proto_library(
    name = "fw_c_proto",
    srcs = proto_files,
)

cc_proto_library(
    name = "fw_cc_proto",
    deps = [":fw_proto"],
)

