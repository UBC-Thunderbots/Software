load("@bazel_embedded//tools/openocd:openocd_repository.bzl", "openocd_deps")

def _openocd_extension_impl(_ctx):
    openocd_deps()

openocd_extension = module_extension(
    implementation = _openocd_extension_impl,
)
