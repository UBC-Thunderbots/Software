#! /bin/bash

# TODO: comment on what this script does and why we need it

LOG_FILE_NAME=profdata_to_lcov.log

# All the folders we'll search for object files in
declare -a OBJECT_FILES_FOLDERS=(
    "bazel-out/k8-fastbuild/bin/software"
    "bazel-out/k8-fastbuild/bin/firmware_new"
    "bazel-out/k8-fastbuild/bin/firmware"
)

# Find all object files
declare -a OBJECT_FILES=()
for folder in "${OBJECT_FILES_FOLDERS[@]}"; do
    for file in $(find -L  $folder -iname "*.o"); do 
         OBJECT_FILES+=( "$file" )
    done
done

# Create a list of arguments like "-o my_file1.o -o my_other_file.o ..." that 
# we can pass into `llvm-cov`
OBJECT_FILES_ARG=""
for object_file in "${OBJECT_FILES[@]}"; do
    OBJECT_FILES_ARG="$OBJECT_FILES_ARG -object $object_file"
done

# Remove the log if it already exists
rm -f $LOG_FILE_NAME

# Find all profdata files (we assume bazel was configured to output profdata
# to all the ".dat" files, there's no easy way to check that it's not just an
# lcov file already), and convert them into lcov files
for profdata_file_name in $(find -L bazel-testlogs -iname "coverage.dat"); do
    output_lcov_file_name="$profdata_file_name.lcov.txt"

    echo "Converting profdata file $profdata_file_name to lcov file: $output_lcov_file_name"

    # Note the file we're converting in the log
    echo "==== $output_lcov_file_name =====" >> $LOG_FILE_NAME

    # Remove the output if it already exists
    rm -f $output_lcov_file_name

    # Generate the lcov data and pipe it into files with the same name but
    # a different extension, saving and errors to a file. We can't just 
    # log all the errors to the shell because of how many we cause by just
    # indiscriminately passing everything we can find to `llvm-cov`.
    # NOTE: it's bad practice to call `llvm-cov` directly like this, but we 
    #       want to be sure that we're using the right version, and if we 
    #       declare this script as a `sh_binary` in bazel, it will overwrite the 
    #       bazel-testlogs folder when it's run (thus overwriting the profdata
    #       files)
    ./bazel-src/external/llvm_clang/bin/llvm-cov export -format=lcov -instr-profile="$profdata_file_name" $OBJECT_FILES_ARG > $output_lcov_file_name 2> $LOG_FILE_NAME
done
