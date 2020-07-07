package(default_visibility = ["//visibility:public"])

load("@rules_python//python:defs.bzl", "py_binary")
load("@python_control_deps//:requirements.bzl", "requirement")

py_binary(
    name = "difference_equation",
    srcs = [":difference_equation.py"],
    main = ":difference_equation.py",
    deps = [
        requirement("numpy"),
        requirement("control"),
        requirement("scipy"),
        requirement("pyparsing"),
        requirement("cycler"),
        requirement("kiwisolver"),
        requirement("python-dateutil"),
    ],
)

py_test(
    name = "discrete_controller_test",
    srcs = [":difference_equation_test.py"],
    main = ":difference_equation_test.py",
    deps = [
        ":difference_equation",
    ],
)

py_binary(
    name = "generate_difference_equation",
    srcs = [":generate_difference_equation.py"],
    main = ":generate_difference_equation.py",
    deps = [
        requirement("control"),
        requirement("pyparsing"),
        requirement("cycler"),
        requirement("kiwisolver"),
        requirement("python-dateutil"),
    ],
)

py_test(
    name = "generate_difference_equation_test",
    srcs = [":generate_difference_equation_test.py"],
    main = ":generate_difference_equation_test.py",
    deps = [
        ":generate_difference_equation",
    ],
)

genrule(
    name = "controller_coefficients_cc_generate",
    outs = [
        "generated_controller_coefficients.h",
    ],
    cmd = "python3 $(location :generate_difference_equation) \
           $(OUTS)",
    tools = [
        ":generate_difference_equation",
    ],
    visibility = ["//visibility:private"],
)
