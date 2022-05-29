load("@rules_proto//proto:defs.bzl", "ProtoInfo")
load("@bazel_tools//tools/cpp:toolchain_utils.bzl", "find_cpp_toolchain")

# The relative filename of the header file.
_FILENAME = "lib/{dirname}/{filename}"

_PROTO_DIR = "{path}/proto"
_OPTIONS_DIR = "{path}/generator/nanopb"

# Command that copies the source to the destination.
_COPY_COMMAND = "cp {source} {destination}"

# Command that zips files recursively. It enters the output directory first so
# that the zipped path starts at lib/.
_ZIP_COMMAND = "cd {output_dir} && zip -qq -r -u {zip_filename} lib/"

def _nanopb_proto_library_impl(ctx):
    # This is the folder we will place all our generation artifacts in
    #
    # Because we need to generate the `.c` and `.h` files for all the proto files required
    # for this library, including all the transitive dependencies, we would like to
    # generate these in the same folder as each proto file. However bazel will not
    # permit us to modify/produce files outside the folder that this rule is called from.
    # We get around this be reproducing the folder structure under a generation folder
    # and adding the root of this directory as an include path.
    #
    generation_folder_name = ctx.attr.name + "_nanopb_gen/"
    generated_folder_abs_path = ctx.genfiles_dir.path + "/" + \
                                ctx.build_file_path[:-len("BUILD")] + generation_folder_name

    # Generate import flags for the protobuf compiler so it can find proto files we
    # depend on, and a list of proto files to include
    all_proto_files = depset()
    all_proto_include_dirs = depset()
    for dep in ctx.attr.deps:
        all_proto_files = depset(
            transitive = [all_proto_files, dep[ProtoInfo].transitive_sources],
        )
        all_proto_include_dirs = depset(
            transitive = [all_proto_include_dirs, dep[ProtoInfo].transitive_proto_path],
        )

    all_proto_hdr_files = []
    all_proto_src_files = []

    for proto_file in all_proto_files.to_list():
        # Generate the `.pb` files using the protobuf compiler
        pb_file_name = generation_folder_name + proto_file.path[:-len(".proto")] + ".pb"
        pb_file = ctx.actions.declare_file(pb_file_name)

        # Create the arguments for the proto compiler to compile the proto file,
        # adding all the transitive include directories
        proto_compile_args = ["-o", pb_file.path, proto_file.path]
        for path in all_proto_include_dirs.to_list():
            proto_compile_args += ["-I%s" % path]

        ctx.actions.run(
            inputs = all_proto_files,
            outputs = [pb_file],
            arguments = proto_compile_args,
            executable = ctx.executable.protoc,
            mnemonic = "ProtoCompile",
            use_default_shell_env = True,
        )

        # Generate the equivalent C code using Nanopb
        h_out_name = generation_folder_name + proto_file.path[:-len(".proto")] + ".nanopb.h"
        c_out_name = generation_folder_name + proto_file.path[:-len(".proto")] + ".nanopb.c"
        c_out = ctx.actions.declare_file(c_out_name)
        h_out = ctx.actions.declare_file(h_out_name)

        ctx.actions.run_shell(
            tools = [ctx.executable.nanopb_generator],
            inputs = [pb_file],
            outputs = [c_out, h_out],
            mnemonic = "NanopbGeneration",
            command = "%s -e .nanopb %s" % (ctx.executable.nanopb_generator.path, pb_file.path),
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
            generated_folder_abs_path,
        ] + [generated_folder_abs_path + dir for dir in all_proto_include_dirs.to_list()],
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

    # platformio_* bazel rules require a provider named transitive_zip_files and an output zip file
    # these contain all files needed for compilation with platformio.
    name = ctx.label.name
    commands = []
    inputs = all_proto_hdr_files + all_proto_src_files
    outputs = []

    for hdr_file in all_proto_hdr_files:
        dir = _PROTO_DIR.format(path = name)
        if "options.nanopb." in hdr_file.basename:
            dir = _OPTIONS_DIR.format(path = name)
        file = ctx.actions.declare_file(
            _FILENAME.format(dirname = dir, filename = hdr_file.basename),
        )
        outputs.append(file)
        commands.append(_COPY_COMMAND.format(
            source = hdr_file.path,
            destination = file.path,
        ))

    for src_file in all_proto_src_files:
        dir = _PROTO_DIR.format(path = name)
        if "options.nanopb." in src_file.basename:
            dir = _OPTIONS_DIR.format(path = name)
        file = ctx.actions.declare_file(
            _FILENAME.format(dirname = dir, filename = src_file.basename),
        )
        outputs.append(file)
        commands.append(_COPY_COMMAND.format(
            source = src_file.path,
            destination = file.path,
        ))

    outputs.append(ctx.outputs.zip)
    commands.append(_ZIP_COMMAND.format(
        output_dir = ctx.outputs.zip.dirname,
        zip_filename = ctx.outputs.zip.basename,
    ))
    ctx.actions.run_shell(
        inputs = inputs,
        outputs = outputs,
        command = "\n".join(commands),
    )

    return struct(
        transitive_zip_files = depset([ctx.outputs.zip]),
        providers = [CcInfo(
            compilation_context = compilation_context,
            linking_context = linking_context,
        )],
    )

nanopb_proto_library = rule(
    implementation = _nanopb_proto_library_impl,
    outputs = {
        "zip": "%{name}.zip",  #output needed to be included in platformio_library
    },
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            providers = [
                ProtoInfo,
            ],
        ),
        "nanopb_libs": attr.label_list(
            providers = [CcInfo],
            default = [Label("@nanopb//:nanopb_header")],
        ),
        "nanopb_generator": attr.label(
            executable = True,
            cfg = "host",
            default = Label("@nanopb//:nanopb_generator"),
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
