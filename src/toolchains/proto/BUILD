load("@rules_proto//proto:defs.bzl", "proto_lang_toolchain")

# Configure protoc to have the right arguments for generating Python stubs.
# NB: the protobuf team intends to remove --python_out and instead use a protoc plugin for Python stub emit.
proto_lang_toolchain(
    name = "protoc_py",
    command_line = "--python_out=%s",
    progress_message = "Generating Python proto_library %{label}",
    runtime = "@thunderscope_deps//protobuf",
    toolchain_type = "@rules_python//python/proto:toolchain_type",
)
