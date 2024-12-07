# Description:
#   PyTorch is a machine learning library for accelerated
#   tensor computing and developing deep neural networks.
#   https://pytorch.org/

cc_library(
    name = "pytorch",
    srcs = [
        "lib/libc10.so",
        "lib/libtorch.so",
        "lib/libtorch_cpu.so",
        "lib/libtorch_global_deps.so",
    ],
    hdrs = glob(["include/**/*.h"]),
    includes = [
        "include",
        "include/torch/csrc/api/include",
    ],
    visibility = ["//visibility:public"],
)
