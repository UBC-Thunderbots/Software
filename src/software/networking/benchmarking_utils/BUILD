package(default_visibility = ["//visibility:public"])

cc_library(
    name = "latency_tester_node",
    srcs = ["latency_tester_node.cpp"],
    hdrs = ["latency_tester_node.h"],
    deps = [
        "//shared:constants",
        "//software/networking:udp_listener",
        "//software/networking:udp_sender",
        "@boost//:asio",
    ],
)

cc_binary(
    name = "latency_tester_primary_node",
    srcs = [
        "latency_tester_primary_node.cpp",
        "latency_tester_primary_node.h",
        "latency_tester_primary_node_main.cpp",
    ],
    deps = [
        ":latency_tester_node",
        "//software/logger:logger",
        "//software/math:statistics_functions",
        "@boost//:program_options",
        "@tracy",
    ],
)

cc_binary(
    name = "latency_tester_secondary_node",
    srcs = [
        "latency_tester_secondary_node.cpp",
        "latency_tester_secondary_node.h",
        "latency_tester_secondary_node_main.cpp",
    ],
    deps = [
        ":latency_tester_node",
        "@boost//:program_options",
    ],
)
