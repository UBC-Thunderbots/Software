package(default_visibility = ["//visibility:public"])

cc_library(
    name = "observer",
    hdrs = [
        "observer.h",
        "observer.tpp",
    ],
    deps = [":thread_safe_buffer"],
)

cc_library(
    name = "subject",
    hdrs = [
        "subject.h",
        "subject.tpp",
    ],
    deps = [
        ":observer",
    ],
)

cc_library(
    name = "thread_safe_buffer",
    hdrs = [
        "thread_safe_buffer.h",
        "thread_safe_buffer.tpp",
    ],
    deps = [
        "//software/logger",
        "//software/time:duration",
        "@boost//:circular_buffer",
    ],
)

cc_library(
    name = "threaded_observer",
    hdrs = [
        "threaded_observer.h",
        "threaded_observer.tpp",
    ],
    deps = [
        ":observer",
        ":thread_safe_buffer",
        "//software/time:duration",
        "@boost//:bind",
    ],
)

cc_test(
    name = "observer_test",
    srcs = ["observer_test.cpp"],
    deps = [
        ":observer",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "subject_test",
    srcs = ["subject_test.cpp"],
    deps = [
        ":subject",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "thread_safe_buffer_test",
    srcs = ["thread_safe_buffer_test.cpp"],
    deps = [
        ":thread_safe_buffer",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "threaded_observer_test",
    srcs = ["threaded_observer_test.cpp"],
    deps = [
        ":threaded_observer",
        "@gtest//:gtest_main",
    ],
)
