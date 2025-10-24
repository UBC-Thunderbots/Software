load("@bazel_tools//tools/cpp:toolchain_utils.bzl", "find_cpp_toolchain")
load("@platformio_rules//platformio:platformio.bzl", "PlatformIOLibraryInfo")
# This file is heavily referencing platformio.bzl from rules_platformio project
# https://github.com/mum4k/platformio_rules

# This rule is an adapter between proto_library + nanopb for the platformio_library
# There is a similar rule: cc_nanopb_proto_library from nanopb natively which does something similar
# However, platformio_rules do not support cc_library due to the way cross compiling works in platformio for
# embedded systems (cc_library compiles srcs to .so/.a files which is not supported the same way by platformio).

load("@rules_proto//proto:defs.bzl", "ProtoInfo")

# The relative filename of the header file.
_FILENAME = "lib/{dirname}/{filename}"

_PROTO_DIR = "{path}/proto"

# Command that makes a directory
_MAKE_DIR_COMMAND = "mkdir -p {dirname}"

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

    # See https://jpa.kapsi.fi/nanopb/docs/reference.html#proto-file-options
    # Section "Defining the options in a .options file" for more information
    # on nanopb option files
    all_options_map = {}
    for opt_file in ctx.files.options:
        opt_basename = opt_file.basename[:-len(".options")]
        all_options_map[opt_basename] = opt_file

    all_proto_hdr_files = []
    all_proto_src_files = []

    for proto_file in all_proto_files.to_list():
        if proto_file.basename in ("descriptor.proto", "nanopb.proto"):
            continue
        proto_compile_args = []
        for path in all_proto_include_dirs.to_list():
            proto_compile_args += ["-I%s" % path]
        target_path = proto_file.path[:-len(".proto")]
        h_out_name = generation_folder_name + target_path + ".nanopb.h"
        c_out_name = generation_folder_name + target_path + ".nanopb.c"
        c_out = ctx.actions.declare_file(c_out_name)
        h_out = ctx.actions.declare_file(h_out_name)

        proto_compile_args += ["--plugin=protoc-gen-nanopb=%s" % (ctx.executable.nanopb_generator.path)]
        proto_compile_args += ["--nanopb_out=%s %s" % (generated_folder_abs_path, proto_file.path)]

        nanopb_opts = ["--extension=.nanopb"]
        proto_basename = proto_file.basename[:-len(".proto")]
        if proto_basename in all_options_map:
            nanopb_opts.append("--options-file=%s" % all_options_map[proto_basename].path)

        for opt in nanopb_opts:
            proto_compile_args += ["--nanopb_opt=%s" % opt]

        cmd = [ctx.executable.protoc.path] + proto_compile_args
        cmd_str = " ".join(cmd)
        ctx.actions.run_shell(
            tools = [
                ctx.executable.protoc,
                ctx.executable.nanopb_generator,
            ],
            inputs = all_proto_files.to_list() + list(all_options_map.values()),
            outputs = [c_out, h_out],
            mnemonic = "NanopbGeneration",
            command = cmd_str,
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

    # Get include paths from Nanopb dependencies
    nanopb_includes = []
    for lib in ctx.attr.nanopb_libs:
        cc_info = lib[CcInfo]
        compilation_context = cc_info.compilation_context

        # Collect all include paths
        nanopb_includes.extend(compilation_context.includes.to_list())
        nanopb_includes.extend(compilation_context.quote_includes.to_list())

    # Create compiler flags
    copts = ["-I{}".format(include) for include in depset(nanopb_includes).to_list()]

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
        user_compile_flags = copts,
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
        file = ctx.actions.declare_file(
            _FILENAME.format(dirname = dir, filename = src_file.basename),
        )
        outputs.append(file)
        commands.append(_COPY_COMMAND.format(
            source = src_file.path,
            destination = file.path,
        ))

    extra_context = cc_common.create_compilation_context(
        includes = depset(["external/nanopb+"]),
        defines = depset(["PB_FIELD_32BIT"]),
    )

    final_compilation_context = cc_common.merge_compilation_contexts(
        compilation_contexts = [compilation_context, extra_context],
    )

    zip_file = ctx.actions.declare_file("%s.zip" % name)
    outputs.append(zip_file)
    commands.append(_ZIP_COMMAND.format(
        output_dir = zip_file.dirname,
        zip_filename = zip_file.basename,
    ))
    ctx.actions.run_shell(
        inputs = inputs,
        outputs = outputs,
        command = "\n".join(commands),
    )
    runfiles = ctx.runfiles(files = [zip_file])
    transitive_libdeps = []
    for dep in ctx.attr.deps:
        if PlatformIOLibraryInfo in dep:
            runfiles.merge_all(dep[PlatformIOLibraryInfo].runfiles)
            transitive_libdeps.extend(dep[PlatformIOLibraryInfo].transitive_libdeps)
    return [
        DefaultInfo(files = depset([zip_file])),
        PlatformIOLibraryInfo(
            default_runfiles = runfiles,
            transitive_libdeps = transitive_libdeps,
        ),
        CcInfo(
            compilation_context = final_compilation_context,
            linking_context = linking_context,
        ),
    ]

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
            default = [Label("@nanopb//:nanopb")],
        ),
        "nanopb_generator": attr.label(
            executable = True,
            cfg = "host",
            default = Label("@nanopb//:protoc-gen-nanopb"),
        ),
        "protoc": attr.label(
            executable = True,
            cfg = "host",
            default = Label("@protobuf//:protoc"),
        ),
        "options": attr.label_list(
            allow_files = [".options"],
            mandatory = False,
        ),
        "_cc_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain"),
        ),
    },
    provides = [
        DefaultInfo,
        PlatformIOLibraryInfo,
        CcInfo,
    ],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
    fragments = ["cpp"],
    host_fragments = ["cpp"],
)
