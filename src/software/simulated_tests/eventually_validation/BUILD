load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "eventually_validation",
    srcs = [
        "robot_enters_region.py",
    ],
    deps = [
        "//proto:software_py_proto",
        "//proto:tbots_py_proto",
        "//software/simulated_tests:validation",
    ],
)
