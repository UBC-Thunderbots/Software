load("@rules_shell//shell:sh_binary.bzl", "sh_binary")

package(default_visibility = ["//visibility:public"])

sh_binary(
    name = "run_autoref",
    srcs = ["run_autoref.sh"],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)
