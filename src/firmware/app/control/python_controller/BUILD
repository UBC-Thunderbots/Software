package(default_visibility = ["//visibility:public"])

load("@rules_python//python:defs.bzl", "py_binary")
load("@python_control_deps//:requirements.bzl", "requirement")

py_binary(
    name = "python_control",
    srcs = ["//firmware/app/control/python_controller/difference_equation.py"],
    deps = [
        requirement("numpy"),
        requirement("control"),
        requirement("scipy"),
    ],
)

#py_test(
#    name = "discrete_controller_test",
#    srcs = ["//firmware/app/control/python_controller/difference_equation_test.py"],
#    deps = [
#        "//firmware/app/control/python_controller:python_control",
#        requirement("numpy"),
#        requirement("control"),
#        requirement("scipy"),
#    ],
#)
