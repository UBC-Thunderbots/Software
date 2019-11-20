load("@rules_proto//proto:defs.bzl", "proto_library", "proto_lang_toolchain", "ProtoInfo")
load("@com_google_protobuf//:protobuf.bzl", "proto_gen")
load("@bazel_skylib//lib:versions.bzl", "versions")
load("@rules_cc//cc:defs.bzl", "cc_library")

def c_proto_library(name, srcs, **kwargs):

    """Creates a cc_library given the proto files.

    nanopb relies on protoc to do the parsing before it converts the files into
    c header/source files. This macro runs protoc to get the desired output, and then
    puts it through the generator script to generate the c/h files (which is then turned into a lib)

    Args:
      name: name of the filegroup that will contain compiled pb files
      srcs: list of proto files

    """
    c_outs = []
    h_outs = []

    for src in srcs:

        proto_file = src.rsplit('/', 1)[-1]
        proto_name = proto_file[:-len(".proto")]

        native.genrule(
                name = "%s_proto_pb_genrule" % (proto_name),
                srcs = [src],
                tools = ["@com_google_protobuf//:protoc",],
                cmd = "protoc -o %s.pb $(location %s) && mv %s.pb $@" % (proto_name, src, proto_name),
                outs = ["%s.pb" % proto_name],
        )

        h_out = src[:-len(".proto")] + ".pb.h"
        c_out = src[:-len(".proto")] + ".pb.c"

        native.genrule(
                name = "%s_proto_ch_genrule" % (proto_name),
                srcs = ["%s.pb" % (proto_name)],
                tools = ["@nanopb//:nanopb_generator",],
                cmd = "python3 $(location @nanopb//:nanopb_generator) $(location %s.pb)" % (proto_name),
                outs = [h_out, c_out],
        )

        c_outs.append(c_out)
        h_outs.append(h_out)

    native.cc_library(
           name = name,
           srcs = c_outs,
           hdrs = h_outs,
           deps = ["@nanopb//:nanopb_header"],
           **kwargs)
