#! /bin/bash

# TODO: comment on what this script does and why we need it

# All the folders we'll search for object files in
OBJECT_FILES_FOLDERS="bazel-out/k8-fastbuild/bin/software bazel-out/k8-fastbuild/bin/firmware bazel-out/k8-fastbuild/bin/firmware_new"

# Find all object files
OBJECT_FILES=""
for folder in $OBJECT_FILES_FOLDERS; do
    OBJECT_FILES="$OBJECT_FILES $(find -L  $folder -iname "*.o")"
done

# Find all profdata files (we assume bazel was configured to output profdata
# to all the ".dat" files, there's no easy way to check that it's not just an
# lcov file already)
PROFDATA_FILES=$(find -L bazel-testlogs -iname "coverage.dat")

# Create a list of arguments like "-o my_file1.o -o my_other_file.o ..." that 
# we can pass into `llvm-cov`
OBJECT_FILES_ARG=""
for object_file in $OBJECT_FILES; do
    OBJECT_FILES_ARG="$OBJECT_FILES_ARG -object $object_file"
done

# Convert all the profdata files into lcov files
for profdata_file_name in $PROFDATA_FILES; do
    output_lcov_file_name="$profdata_file_name.lcov.txt"

    echo "Converting profdata file $profdata_file_name to lcov file: $output_lcov_file_name"

    # Remove the output if it already exists
    rm -f $output_lcov_file_name

    # Generate the lcov files for the given profdata
    # NOTE: it's bad practice to call `llvm-cov` like this, but we want to be
    #       sure that we're using the right version, and if we declare this
    #       script as a `sh_binary` in bazel, it will overrite the 
    #       bazel-testlogs folder when it's run (thus overwriting the profdata
    #       files)
    ./bazel-src/external/llvm_clang/bin/llvm-cov export -format=lcov -instr-profile="$profdata_file_name" $OBJECT_FILES_ARG > $output_lcov_file_name 
    #> /dev/null 2>&1
done
