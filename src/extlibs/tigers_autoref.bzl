# Description:
#   TIGERs AutoReferee is an automatic referee for RoboCup Small Size League.
#   https://github.com/TIGERs-Mannheim/AutoReferee
#
#   This BUILD overlay exposes the prebuilt distribution downloaded by
#   http_archive in MODULE.bazel. 
#
#   This target provides runtime files only. Starting AutoRef with a
#   Bazel-managed Java runtime is handled separately.

load("@rules_java//java:java_import.bzl", "java_import")

java_import(
    name = "jars",
    jars = glob(["lib/*.jar"]),
    visibility = ["//visibility:public"],
)

AUTOREF_RUNTIME_FILES = [
    # Upstream launch scripts.
    "bin/**",
    # Configuration files used by AutoRef at runtime.
    "config/**",
    # Application and dependency JARs used by the upstream launcher.
    "lib/**",
]

filegroup(
    name = "distribution",
    srcs = glob(AUTOREF_RUNTIME_FILES),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "config",
    srcs = glob(["config/**"]),
    visibility = ["//visibility:public"],
)