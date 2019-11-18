load("@rules_proto//proto:defs.bzl", "proto_library", "proto_lang_toolchain")
load(":nanopb.bzl", "c_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
        name = "fw_proto",
        srcs = ["robot.proto",],
        visibility = ["//visibility:private"]
)

c_proto_library(
    name = "fw_c_proto",
    deps = [":fw_proto"],
)

#genrule(
    #name = "generate_pb_files",
    #srcs = ["robot.proto"],
    #tools = [
         #"@com_google_protobuf//:protoc",
    #],
    #cmd = "protoc -o robot.pb $(location robot.proto) && mv robot.pb $@",
    #outs = ["robot.pb"],
#)

#genrule(
    #name = "generate_c_and_h",
    #srcs = [":generate_pb_files"],
    #tools = [
        #"@nanopb//:nanopb_generator",
    #],
    #cmd = "",
#)


