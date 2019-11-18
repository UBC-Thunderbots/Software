load("@rules_proto//proto:defs.bzl", "proto_library", "proto_lang_toolchain", "ProtoInfo")
load("@com_google_protobuf//:protobuf.bzl", "proto_gen")
load("@bazel_skylib//lib:versions.bzl", "versions")
load("@rules_cc//cc:defs.bzl", "cc_library")

def getHeaders(srcs):
    return [src[:-len(".proto")] + ".pb.h" for src in srcs]

def getSources(srcs):
    return [src[:-len(".proto")] + ".pb.c" for src in srcs]

def c_proto_library(name, srcs, **kwargs):

    """nanopb relies on protoc to do the parsing before it converts the files into
    c header/source files. This macro creates a filegroup w/ all of of the converted pb files

    Args:
      name: name of the filegroup that will contain compiled pb files
      srcs: list of proto files

    """
    pb_outs = []
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

        native.genrule(
                name = "%s_proto_ch_genrule" % (proto_name),
                srcs = ["%s.pb" % (proto_name)],
                tools = ["@nanopb//:nanopb_generator",],
                cmd = "ls -R && python3 $(location @nanopb//:nanopb_generator) $(location %s.pb)" % (proto_name),
                outs = getSources([src]) + getHeaders([src])
                )

    native.filegroup(
           name = name,
           srcs = getSources([src]) + getHeaders([src]),
           **kwargs)
