load("@rules_proto//proto:defs.bzl", "ProtoInfo", "proto_library")
load("@com_google_protobuf//:protobuf.bzl", "proto_gen")
load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_cc//cc:toolchain_utils.bzl", "find_cpp_toolchain")

# TODO: move this to `external/nanopb.bzl`

# TODO: rename this file to `rules.bzl` to be consistent with other parts of the repo?

# TODO: this should probably eventually be a seperate "nanopb_rules" repository so we
#       don't have to seperately include nanopb and pass it into this rule. Maybe
#       open pull request to the nanopb repository?

def _nanopb_proto_library_impl(ctx):
    # Get all the proto files recursively from the dependencies of this rule
    proto_infos = [dep[ProtoInfo] for dep in ctx.attr.deps]
    all_proto_files = depset()
    for proto_info in proto_infos:
        all_proto_files = depset(transitive = [all_proto_files, proto_info.transitive_sources])

    # For each proto file, generate the equivalent C code using nanopb
    all_proto_hdr_files = []
    all_proto_src_files = []

    for proto_file in all_proto_files.to_list():
        #native.genrule(
        #    name = "%s_proto_pb_genrule" % (proto_name),
        #    srcs = [src],
        #    tools = ["@com_google_protobuf//:protoc"],
        #    cmd = "protoc -o %s.pb $(location %s) && mv %s.pb $@" % (proto_name, src, proto_name),
        #    outs = ["%s.pb" % proto_name],
        #)
        #native.genrule(
        #    name = "%s_proto_ch_genrule" % (proto_name),
        #    srcs = ["%s.pb" % (proto_name)],
        #    tools = ["@nanopb//:nanopb_generator"],
        #    cmd = "python3 $(location @nanopb//:nanopb_generator) $(location %s.pb)" % (proto_name),
        #    outs = [h_out, c_out],
        #)

        # TODO: will this breakdown if the proto file is not in the current directly
        pb_file_name = proto_file.basename.strip(".proto") + ".pb"
        pb_file = ctx.actions.declare_file(pb_file_name)

        # TODO: implement me!
        # TODO: go over all fields and add if good
        # TODO: include directories (should just be the root bazel directory?)
        # TODO: we should be using the given proto compiler here
        ctx.actions.run_shell(
            inputs = [proto_file],
            outputs = [pb_file],
            command = "protoc -o %s %s" % (pb_file.path, proto_file.path),
        )

        # TODO: is this gonna cause bugs if the proto file isn't in the current directory?
        h_out_name = proto_file.basename.strip("\.proto") + ".pb.h"
        c_out_name = proto_file.basename.strip("\.proto") + ".pb.c"

        c_out = ctx.actions.declare_file(c_out_name)
        h_out = ctx.actions.declare_file(h_out_name)

        # TODO: REMOVE ALL PRINT STATEMENTS
        print(ctx.attr.nanopb_generator)
        print(ctx.executable.nanopb_generator)
        print(pb_file.path)
        ctx.actions.run_shell(
            tools = [ctx.executable.nanopb_generator],
            inputs = [pb_file],
            outputs = [c_out, h_out],
            command = "%s %s" % (ctx.executable.nanopb_generator.path, pb_file.path),
        )

        all_proto_src_files.append(c_out)
        all_proto_hdr_files.append(h_out)

    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
    )

    (compilation_context, compilation_outputs) = cc_common.compile(
        name = "compile_nanopb_outputs",
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        srcs = all_proto_src_files,
        public_hdrs = all_proto_hdr_files,
    )

    (linking_context, linking_outputs) = \
        cc_common.create_linking_context_from_compilation_outputs(
            name = "link_nanopb_outputs",
            compilation_outputs = compilation_outputs,
            actions = ctx.actions,
            feature_configuration = feature_configuration,
            cc_toolchain = cc_toolchain,
        )

    return CcInfo(
        compilation_context = compilation_context,
        linking_context = linking_context,
    )

# TODO: delete
#    return native.cc_library(
#        name = ctx.attr.name,
#        srcs = all_proto_src_files,
#        hdrs = all_proto_hdr_files,
#        deps = ctx.attr.nanopb_srcs,
#    )

nanopb_proto_library = rule(
    implementation = _nanopb_proto_library_impl,
    # TODO: doc comments for each attribute
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            # TODO: figure out what provider to specify here to restrict to things that
            #       provide [ProtoInfo]. See: https://github.com/bazelbuild/bazel/issues/6901
            providers = [
                ProtoInfo,
            ],
        ),
        # TODO: more strict requirements on this attr
        "nanopb_srcs": attr.label_list(mandatory = True, providers = [CcInfo]),
        # TODO: more strict requirements on this attr
        "nanopb_generator": attr.label(mandatory = True, executable = True, cfg = "host"),
        "protoc_compiler": attr.label(mandatory = True, executable = True, cfg = "host"),
        "_cc_toolchain": attr.label(default = Label("@bazel_tools//tools/cpp:current_cc_toolchain")),
    },
    provides = [
        # TODO: uncomment this, not working, but it should......
        #        "CcInfo",
    ],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
    fragments = ["cpp"],
    host_fragments = ["cpp"],
)

# TODO: delete this
def c_proto_library_old(name, srcs, **kwargs):
    """Creates a cc_library given the proto files.

    nanopb relies on protoc to do the parsing before it converts the files into
    c header/source files. This macro runs protoc to get the desired output, and then
    puts it through the generator script to generate the c/h files (which is then turned into a lib)

    NOTE: we append _fw so that the files generated here do not conflict
    with the files generated by cc_proto_library for the software proto lib

    Args:
      name: name of the filegroup that will contain compiled pb files
      srcs: list of proto files

    """
    c_outs = []
    h_outs = []

    for src in srcs:
        proto_file = src.rsplit("/", 1)[-1]
        proto_name = proto_file[:-len(".proto")] + "_fw"

        native.genrule(
            name = "%s_proto_pb_genrule" % (proto_name),
            srcs = [src],
            tools = ["@com_google_protobuf//:protoc"],
            cmd = "protoc -o %s.pb $(location %s) && mv %s.pb $@" % (proto_name, src, proto_name),
            outs = ["%s.pb" % proto_name],
        )

        h_out = proto_name + ".pb.h"
        c_out = proto_name + ".pb.c"

        native.genrule(
            name = "%s_proto_ch_genrule" % (proto_name),
            srcs = ["%s.pb" % (proto_name)],
            tools = ["@nanopb//:nanopb_generator"],
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
        **kwargs
    )
