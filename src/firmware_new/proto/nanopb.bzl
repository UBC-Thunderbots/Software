load("@rules_proto//proto:defs.bzl", "proto_library", "proto_lang_toolchain", "ProtoInfo")
load("@bazel_skylib//lib:versions.bzl", "versions")
load("@rules_cc//cc:defs.bzl", "cc_library")

def _proto_c_library_impl(ctx):

    """nanopb relies on protoc to do the parsing before it converts the files into
    c header/source files. Creates a filegroup w/ all of of the converted pb files
    
    NOTE: for internal use only

    Args:
      name: name of the filegroup that contains compiled pb files
      srcs: list of proto files

    """
    pb_outs = []
    srcs = []

    for dep in ctx.attr.deps:
        srcs += [proto_file.path for proto_file in dep[ProtoInfo].check_deps_sources.to_list()]

    for src in srcs:

        proto_file = src.rsplit('/', 1)[-1]
        proto_name = proto_file[:-len(".proto")]

        native.genrule(
                name = "%s_proto_genrule" % (proto_name),
                srcs = [src],
                tools = ["@com_google_protobuf//:protoc",],
                cmd = "protoc -o %s.pb $(location %s) && mv %s.pb $@" % (proto_name, src, proto_name),
                outs = ["%s.pb" % proto_name],
                )

        pb_outs.append("%s.pb" % proto_name)

    native.filegroup(
            name = ctx.attr.name,
            srcs = pb_outs,
    )

c_proto_library = rule(
    attrs = {
        "deps": attr.label_list(),
    },
    implementation = _proto_c_library_impl,
)

#def _proto_c_library_impl(ctx):

    #for src in srcs:
        #args = []

        #in_gen_dir = src.root.path == gen_dir
        #if in_gen_dir:
            #import_flags_real = []
            #for f in depset(import_flags).to_list():
                #path = f.replace("-I", "")
                #import_flags_real.append("-I$(realpath -s %s)" % path)

        #outs = []
        #use_grpc_plugin = (ctx.attr.plugin_language == "grpc" and ctx.attr.plugin)
        #path_tpl = "$(realpath %s)" if in_gen_dir else "%s"
        #if ctx.attr.gen_cc:
            #args += [("--cpp_out=" + path_tpl) % gen_dir]
            #outs.extend(_CcOuts([src.basename], use_grpc_plugin = use_grpc_plugin))
        #if ctx.attr.gen_py:
            #args += [("--python_out=" + path_tpl) % gen_dir]
            #outs.extend(_PyOuts([src.basename], use_grpc_plugin = use_grpc_plugin))

        #outs = [ctx.actions.declare_file(out, sibling = src) for out in outs]
        #inputs = [src] + deps
        #tools = [ctx.executable.protoc]
        #if ctx.executable.plugin:
            #plugin = ctx.executable.plugin
            #lang = ctx.attr.plugin_language
            #if not lang and plugin.basename.startswith("protoc-gen-"):
                #lang = plugin.basename[len("protoc-gen-"):]
            #if not lang:
                #fail("cannot infer the target language of plugin", "plugin_language")

            #outdir = "." if in_gen_dir else gen_dir

            #if ctx.attr.plugin_options:
                #outdir = ",".join(ctx.attr.plugin_options) + ":" + outdir
            #args += [("--plugin=protoc-gen-%s=" + path_tpl) % (lang, plugin.path)]
            #args += ["--%s_out=%s" % (lang, outdir)]
            #tools.append(plugin)

        #if not in_gen_dir:
            #ctx.actions.run(
                #inputs = inputs,
                #tools = tools,
                #outputs = outs,
                #arguments = args + import_flags + [src.path],
                #executable = ctx.executable.protoc,
                #mnemonic = "ProtoCompile",
                #use_default_shell_env = True,
            #)
        #else:
            #for out in outs:
                #orig_command = " ".join(
                    #["$(realpath %s)" % ctx.executable.protoc.path] + args +
                    #import_flags_real + ["-I.", src.basename],
                #)
                #command = ";".join([
                    #'CMD="%s"' % orig_command,
                    #"cd %s" % src.dirname,
                    #"${CMD}",
                    #"cd -",
                #])
                #generated_out = "/".join([gen_dir, out.basename])
                #if generated_out != out.path:
                    #command += ";mv %s %s" % (generated_out, out.path)
                #ctx.actions.run_shell(
                    #inputs = inputs,
                    #outputs = [out],
                    #command = command,
                    #mnemonic = "ProtoCompile",
                    #tools = tools,
                    #use_default_shell_env = True,
                #)

    #return struct(
        #proto = struct(
            #srcs = srcs,
            #import_flags = import_flags,
            #deps = deps,
        #),
    #)
    

