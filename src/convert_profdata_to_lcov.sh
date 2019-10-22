#! /bin/bash

# TODO: comment on what this script does and why we need it

OBJECT_FILES=$(find -L bazel-bin -iname "*.o")

PROFDATA_FILES=$(find -L bazel-testlogs -iname "*.dat")

## Convert all the profdata (*.dat) files into lcov ones
#find -L bazel-testlogs -iname "*.dat" -type f -exec sh -c '
#    profdata_file_name="$1"
#    output_lcov_file_name="$profdata_file_name.lcov.txt"
#
#    # Remove the output if it already exists
#    rm -f $output_lcov_file_name
#
#    # Generate the output
#    ./bazel-src/external/llvm_clang/bin/llvm-cov export -format=lcov -instr-profile="$profdata_file_name" $OBJECT_FILES > $output_lcov_file_name
#' find-sh {} +

for object_file in $OBJECT_FILES; do
    output_lcov_file_name="$profdata_file_name.lcov.txt"

    echo "Converting profdata file $profdata_file_name to lcov file: $output_lcov_file_name"

    # Remove the output if it already exists
    rm -f $output_lcov_file_name

    # Generate the output
    ./bazel-src/external/llvm_clang/bin/llvm-cov export -format=lcov -instr-profile="$profdata_file_name" $OBJECT_FILES > $output_lcov_file_name
done
