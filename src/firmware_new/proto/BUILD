load("@rules_proto//proto:defs.bzl", "proto_library", "proto_lang_toolchain")
load("@com_google_protobuf//:protobuf.bzl", "proto_gen")

package(default_visibility = ["//visibility:public"])

# proto files
proto_library(
    name = "fw_proto",
    srcs = [
        "robot.proto",
    ],
    deps = [
        "@com_google_protobuf//:descriptor_proto"
    ],
    visibility = ["//visibility:private"]
)


filegroup(
        name = "proto_files",
        srcs = glob(["firmware_new/proto/*.proto"]),
)

genrule(
    name = "generate_c_proto",
    srcs = [
		"robot.proto",
        "//firmware_new/proto:proto_files",
    ],
    outs = [
        "robot.cc",
        "robot.h",
    ],
    cmd = "$(location @nanopb//:nanopb-protoc) --nanopb_out=.  $(location robot.proto)",
	tools = [
		"@nanopb//:nanopb-protoc",
		],
    visibility = ["//visibility:private"],
)
