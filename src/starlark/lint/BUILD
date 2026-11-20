load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("@rules_python//python:py_binary.bzl", "py_binary")
load("@rules_python//python/entry_points:py_console_script_binary.bzl", "py_console_script_binary")

package(default_visibility = ["//visibility:public"])

compile_pip_requirements(
    name = "requirements",
    src = "requirements.in",
    requirements_txt = "requirements_lock.txt",
)

py_console_script_binary(
    name = "ansible_lint",
    pkg = "@lint_deps//ansible_lint",
    script = "ansible-lint",
)

py_console_script_binary(
    name = "clang_format",
    pkg = "@lint_deps//clang_format",
    script = "clang-format",
)

py_console_script_binary(
    name = "codespell",
    pkg = "@lint_deps//codespell",
    script = "codespell",
)

py_console_script_binary(
    name = "md_toc",
    pkg = "@lint_deps//md_toc",
    script = "md_toc",
)

py_binary(
    name = "ruff",
    srcs = ["run_ruff.py"],
    data = ["@lint_deps//ruff:data"],
    main = "run_ruff.py",
    deps = ["@lint_deps//ruff"],
)
