def _generate_dynamic_parameters_impl(ctx):
    output_header = ctx.outputs.generated_parameter_header
    output_source = ctx.outputs.generated_parameter_source

    args = ctx.actions.args()
    args.add("--output_header", output_header)
    args.add("--output_source", output_source)
    args.add("-{}".format(ctx.attr.output_language))

    # Retrieve paths to the header files directly listed in each enum_dep
    # These paths are relative to the bazel WORKSPACE root
    enum_headers = [f for target in ctx.attr.enum_deps for f in target[CcInfo].compilation_context.direct_headers]
    enum_header_paths = [f.path for f in enum_headers]
    args.add_all("--include_headers", enum_headers)

    ctx.actions.run(
        outputs = [output_header, output_source],
        arguments = [args],
        progress_message = "Generating DynamicParameters",
        executable = ctx.executable._generation_script,
    )

    return [DefaultInfo(files = depset([output_header, output_source]))]

generate_dynamic_parameters = rule(
    implementation = _generate_dynamic_parameters_impl,
    attrs = {
        "enum_deps": attr.label_list(),
        "_generation_script": attr.label(
            default = Label("//shared/parameter/generation_scripts:generate_dynamic_parameters"),
            executable = True,
            cfg = "host",
        ),
        "generated_parameter_header": attr.output(mandatory = True),
        "generated_parameter_source": attr.output(mandatory = True),
        "output_language": attr.string(
            mandatory = True,
            values = ["c", "cpp"],
        ),
    },
)

def c_dynamic_parameters(name, generated_parameter_header, generated_parameter_source):
    generate_dynamic_parameters(
        name = name,
        generated_parameter_header = generated_parameter_header,
        generated_parameter_source = generated_parameter_source,
        output_language = "c",
    )

def cpp_dynamic_parameters(name, generated_parameter_header, generated_parameter_source, enum_deps = []):
    generate_dynamic_parameters(
        name = name,
        generated_parameter_header = generated_parameter_header,
        generated_parameter_source = generated_parameter_source,
        enum_deps = enum_deps,
        output_language = "cpp",
    )
