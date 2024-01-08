package(default_visibility = ["//visibility:public"])

cc_library(
    name = "observer",
    hdrs = [
        "observer.hpp",
    ],
    deps = [
        ":thread_safe_buffer",
        "//shared:constants",
    ],
)

cc_library(
    name = "subject",
    hdrs = [
        "subject.hpp",
    ],
    deps = [
        ":observer",
    ],
)

cc_library(
    name = "observer_subject_adapter",
    hdrs = [
        "observer_subject_adapter.hpp",
    ],
    deps = [
        ":observer",
    ],
)

cc_library(
    name = "thread_safe_buffer",
    hdrs = [
        "thread_safe_buffer.hpp",
    ],
    deps = [
        "//software/logger",
        "//software/time:duration",
        "//software/util/typename",
        "@boost//:circular_buffer",
    ],
)

cc_library(
    name = "threaded_observer",
    hdrs = [
        "first_in_first_out_threaded_observer.h",
        "last_in_first_out_threaded_observer.h",
        "threaded_observer.hpp",
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
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "subject_test",
    srcs = ["subject_test.cpp"],
    deps = [
        ":subject",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "observer_subject_adapter_test",
    srcs = ["observer_subject_adapter_test.cpp"],
    deps = [
        ":observer",
        ":observer_subject_adapter",
        ":subject",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "thread_safe_buffer_test",
    srcs = ["thread_safe_buffer_test.cpp"],
    deps = [
        ":thread_safe_buffer",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "first_in_first_out_threaded_observer_test",
    srcs = ["first_in_first_out_threaded_observer_test.cpp"],
    deps = [
        ":threaded_observer",
        "//shared/test_util:tbots_gtest_main",
    ],
    # Run the test up to 3 times if it fails
    flaky = True,
)

cc_test(
    name = "last_in_first_out_threaded_observer_test",
    srcs = ["last_in_first_out_threaded_observer_test.cpp"],
    deps = [
        ":threaded_observer",
        "//shared/test_util:tbots_gtest_main",
    ],
)
