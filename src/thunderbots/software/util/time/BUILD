cc_library(
    name = "duration",
    srcs = ["duration.cpp"],
    hdrs = ["duration.h"],
    deps = [
        ":time",
        "//:shared",
    ],
)

cc_library(
    name = "time",
    srcs = ["time.cpp"],
    hdrs = ["time.h"],
    deps = [],
)

cc_library(
    name = "timestamp",
    srcs = ["timestamp.cpp"],
    hdrs = ["timestamp.h"],
    deps = [":time"],
)

cc_test(
    name = "duration_test",
    srcs = ["duration_test.cpp"],
    deps = [":duration"],
)

cc_test(
    name = "time_test",
    srcs = ["time_test.cpp"],
    deps = [":time"],
)

cc_test(
    name = "timestamp_test",
    srcs = ["timestamp_test.cpp"],
    deps = [":timestamp"],
)
