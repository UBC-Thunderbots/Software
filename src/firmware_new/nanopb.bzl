load("@rules_proto//proto:defs.bzl", "ProtoInfo")
load("@rules_cc//cc:toolchain_utils.bzl", "find_cpp_toolchain")

def _nanopb_proto_library_impl(ctx):
    # This is the folder we will place all our generation artifacts in
    generation_folder_name = ctx.attr.name + "_nanopb_gen/"

    # Generate import flags for the protobuf compiler so it can find proto files we
    # depend on, and a list of proto files to include
    all_proto_files = depset()
    for dep in ctx.attr.deps:
        all_proto_files = depset(
            transitive = [all_proto_files, dep[ProtoInfo].transitive_sources],
        )

    all_proto_hdr_files = []
    all_proto_src_files = []

    for proto_file in all_proto_files.to_list():
        # Generate the `.pb` files using the protobuf compiler
        pb_file_name = generation_folder_name + proto_file.path[:-len(".proto")] + ".pb"
        pb_file = ctx.actions.declare_file(pb_file_name)

        ctx.actions.run(
            inputs = all_proto_files,
            outputs = [pb_file],
            arguments = ["-o", pb_file.path, proto_file.path],
            executable = ctx.executable.protoc,
            mnemonic = "ProtoCompile",
            use_default_shell_env = True,
        )

        # Generate the equivalent C code using Nanopb
        h_out_name = generation_folder_name + proto_file.path[:-len(".proto")] + ".pb.h"
        c_out_name = generation_folder_name + proto_file.path[:-len(".proto")] + ".pb.c"
        c_out = ctx.actions.declare_file(c_out_name)
        h_out = ctx.actions.declare_file(h_out_name)

        ctx.actions.run_shell(
            tools = [ctx.executable.nanopb_generator],
            inputs = [pb_file],
            outputs = [c_out, h_out],
            mnemonic = "NanopbGeneration",
            command = "%s %s" % (ctx.executable.nanopb_generator.path, pb_file.path),
        )

        all_proto_src_files.append(c_out)
        all_proto_hdr_files.append(h_out)

    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
    )

    # Get the compilation and linking contexts from all nanopb srcs
    nanopb_compilation_contexts = [
        label[CcInfo].compilation_context
        for label in ctx.attr.nanopb_libs
        if label[CcInfo].compilation_context != None
    ]
    nanopb_linking_contexts = [
        label[CcInfo].linking_context
        for label in ctx.attr.nanopb_libs
        if label[CcInfo].linking_context != None
    ]

    (compilation_context, compilation_outputs) = cc_common.compile(
        name = "compile_nanopb_outputs",
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        srcs = all_proto_src_files,
        public_hdrs = all_proto_hdr_files,
        includes = [
            ctx.genfiles_dir.path + "/" + ctx.build_file_path[:-len("BUILD")] +
            generation_folder_name,
        ],
        compilation_contexts = nanopb_compilation_contexts,
    )

    (linking_context, linking_outputs) = \
        cc_common.create_linking_context_from_compilation_outputs(
            name = "link_nanopb_outputs",
            compilation_outputs = compilation_outputs,
            actions = ctx.actions,
            feature_configuration = feature_configuration,
            cc_toolchain = cc_toolchain,
            linking_contexts = nanopb_linking_contexts,
        )

    return CcInfo(
        compilation_context = compilation_context,
        linking_context = linking_context,
    )

nanopb_proto_library = rule(
    implementation = _nanopb_proto_library_impl,
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            providers = [
                ProtoInfo,
            ],
        ),
        "nanopb_libs": attr.label_list(
            providers = [CcInfo],
            default = [Label("@com_github_nanopb_nanopb//:nanopb_header")],
        ),
        "nanopb_generator": attr.label(
            executable = True,
            cfg = "host",
            default = Label("@com_github_nanopb_nanopb//:nanopb_generator"),
        ),
        "protoc": attr.label(
            executable = True,
            cfg = "host",
            default = Label("@com_google_protobuf//:protoc"),
        ),
        "_cc_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain"),
        ),
    },
    provides = [
        CcInfo,
    ],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
    fragments = ["cpp"],
    host_fragments = ["cpp"],
)
