package(default_visibility = ["//visibility:public"])

# The prebuilt messense aarch64-unknown-linux-gnu toolchain (Homebrew), exposed
# as Bazel inputs so its binaries (gcc, ld, as, ...) are staged into the sandbox
# when cross-compiling robot firmware on macOS.
filegroup(
    name = "everything",
    srcs = glob(["**"]),
)
