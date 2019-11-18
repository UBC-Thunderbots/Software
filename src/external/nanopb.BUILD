licenses(["notice"])

exports_files(["LICENSE.txt"])

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "nanopb",
  visibility = ["//visibility:public"],
  hdrs = [
    "pb.h",
    "pb_common.h",
    "pb_decode.h",
    "pb_encode.h",
  ],
  srcs = [
    "pb_common.c",
    "pb_decode.c",
    "pb_encode.c",
  ],
)

# this library is linked with all c_proto_libraries
# the generated h and c files access pb.h relatively, so
# we strip external/nanopb to comply

cc_library(
  name = "nanopb_header",
  hdrs = [
	"pb.h",
  ],
  strip_include_prefix = "",
  visibility = ["//visibility:public"],
)

py_binary(
    name = "nanopb_generator",
    srcs = ["generator/nanopb_generator.py"],
    imports = ["proto"],
)
