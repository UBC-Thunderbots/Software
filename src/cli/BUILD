load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("@rules_python//python:py_binary.bzl", "py_binary")
load("@rules_python//python:py_library.bzl", "py_library")
load("@tbots_cli_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

compile_pip_requirements(
    name = "requirements",
    src = "requirements.in",
    requirements_txt = "requirements_lock.txt",
)

py_library(
    name = "cli_lib",
    srcs = [
        "action_argument.py",
        "cli_params.py",
        "fuzzy_target.py",
        "multi_option.py",
    ],
    deps = [
        requirement("click"),
        requirement("iterfzf"),
        requirement("python-levenshtein"),
        requirement("questionary"),
        requirement("thefuzz"),
        requirement("typer"),
    ],
)

py_binary(
    name = "tbots",
    srcs = ["tbots_cli.py"],
    main = "tbots_cli.py",
    deps = [
        ":cli_lib",
        requirement("typer"),
    ],
)
