package(default_visibility = ["//visibility:public"])

genrule(
    name = "copy_csv",
    srcs = ["single_include/csv.hpp"],
    outs = ["csv.hpp"],
    cmd = "cp $< $(@)",
)

cc_library(
    name = "csv",
    hdrs = ["csv.hpp"],
)
