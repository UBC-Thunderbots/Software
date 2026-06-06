load("@gameplay_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "movement_robot_field_test",
    srcs = [
        "movement_robot_field_test.py",
    ],
    # TODO (#2619) Remove tag to run in parallel
    tags = [
        "exclusive",
    ],
    deps = [
        "//software:conftest",
        "//software/gameplay_tests:util",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)

py_test(
    name = "pivot_kick_field_test",
    srcs = [
        "pivot_kick_field_test.py",
    ],
    # TODO (#2619) Remove tag to run in parallel
    tags = [
        "exclusive",
    ],
    deps = [
        "//software:conftest",
        "//software/gameplay_tests:util",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)

py_test(
    name = "passing_field_test",
    srcs = [
        "passing_field_test.py",
    ],
    deps = [
        "//software:conftest",
        "//software/gameplay_tests:util",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)
