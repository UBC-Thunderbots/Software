#! /bin/bash

#
# This script finds `.dat` files, assumes they are `profdata` files, and
# converts them to a `.lcov` file that CodeCov can understand. 
# 
# This script exists because clang outputs a different format for code coverage
# data then gcc does, and bazel has not yet unified it's coverage output.
# Once https://github.com/bazelbuild/bazel/issues/9406 is resolved we should be
# able to remove this script
#

MERGED_PROFDATA_FILE_NAME="merged_coverage.dat"
MERGED_LCOV_FILE_NAME="$MERGED_PROFDATA_FILE_NAME.lcov"
PROFDATA_FILE_LIST_NAME="all_coverage_files.txt"

# All the folders we'll search for object files in
declare -a OBJECT_FILES_FOLDERS=(
    "bazel-out/k8-fastbuild/bin/software"
    "bazel-out/k8-fastbuild/bin/firmware_new"
    "bazel-out/k8-fastbuild/bin/firmware"
)

# Some object files seem to make `llvm-cov` fail with the following error:
# "error: Could not load coverage information"
# So we ignore them.
# At the time of writing, the known bad files are:
#   bazel-out/k8-fastbuild/bin/software/visualizer/widgets/_objs/parameters/moc_parameters.pic.o
#   bazel-out/k8-fastbuild/bin/software/visualizer/widgets/_objs/robot_status/moc_robot_status.pic.o
#   bazel-out/k8-fastbuild/bin/software/visualizer/widgets/_objs/world_view/moc_world_view.pic.o
#   bazel-out/k8-fastbuild/bin/software/visualizer/widgets/_objs/ai_control/moc_ai_control.pic.o
# Please refer to this post for how to construct a prune argument. It's not
# as simple as you think. https://stackoverflow.com/a/4210072
BLACKLIST_ARG="-path bazel-out/k8-fastbuild/bin/software/visualizer/widgets -prune -o"

# Find all object files
declare -a OBJECT_FILES=()
for folder in "${OBJECT_FILES_FOLDERS[@]}"; do
    for file in $(find -L $folder $BLACKLIST_ARG -name '*.o' -type f -print); do 
        OBJECT_FILES+=( "$file" )
    done
done

# Create a list of arguments like "-o my_file1.o -o my_other_file.o ..." that 
# we can pass into `llvm-cov`
OBJECT_FILES_ARG=""
for object_file in "${OBJECT_FILES[@]}"; do
    OBJECT_FILES_ARG="$OBJECT_FILES_ARG -object $object_file"
done

# Find all profdata files (we assume bazel was configured to output profdata
# to all the ".dat" files, there's no easy way to check that it's not just an
# lcov file already), and put the found paths into a file
# Create a file with the names of all the coverage files we want to merge
rm -rf $PROFDATA_FILE_LIST_NAME
for profdata_file_path in $(find -L bazel-testlogs -iname "coverage.dat"); do
    echo "$profdata_file_path" >> $PROFDATA_FILE_LIST_NAME
done

# Merge all the profdata files into one big one
rm -rf $MERGED_PROFDATA_FILE_NAME
./bazel-src/external/llvm_clang/bin/llvm-profdata merge -output=$MERGED_PROFDATA_FILE_NAME -input-files=$PROFDATA_FILE_LIST_NAME

# Convert the merged profdata file into a single lcov file
rm -rf $MERGED_LCOV_FILE_NAME
./bazel-src/external/llvm_clang/bin/llvm-cov export -format=lcov -instr-profile="$MERGED_PROFDATA_FILE_NAME" $OBJECT_FILES_ARG > "$MERGED_PROFDATA_FILE_NAME.lcov"

# Clean up intermediate files, otherwise CodeCov will pick these up as well
rm -rf $PROFDATA_FILE_LIST_NAME
rm -rf $MERGED_PROFDATA_FILE_NAME

