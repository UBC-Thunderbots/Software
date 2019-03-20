
# given a list of files and the path to them, this constructs absolute paths to each of those files.
# the given outname is the name of the return variable
function(set_src_destinations "src_files" "dest" "outname")
    set(local_src_w_paths)
    # loop through each file and build its path
    foreach("file" ${src_files})
        list(APPEND local_src_w_paths "${dest}/${file}")
    endforeach("file")
    # do a pseudo-return by setting the outname variable to the local list
    set(${outname} "${local_src_w_paths}" PARENT_SCOPE)
endfunction(set_src_destinations)

# set the location of the firmware and test directories
set(FIRMWARE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(TEST_SOURCE_DIR "${FIRMWARE_SOURCE_DIR}/test")

# set the name of the binary file
set(BINARY_NAME "fw_test")

# tell cmake to run our generator bash script
# remove the test.c file and then regenerate it based on the test files in the unit_tests/ folder
execute_process(
        COMMAND rm -f ${TEST_SOURCE_DIR}/test.c
        COMMAND bash ${TEST_SOURCE_DIR}/test_generator.sh ${TEST_SOURCE_DIR})

# innclude the build_firmware.cmake file so we can use its set_include_dirs function
include("${FIRMWARE_SOURCE_DIR}/cmake_binary_builders/build_firmware.cmake")

# include all the header directories
set_include_dirs(${FIRMWARE_SOURCE_DIR})
include_directories(${TEST_SOURCE_DIR}/../main)
include_directories(${TEST_SOURCE_DIR})
include_directories(${TEST_SOURCE_DIR}/unit_tests)
include_directories(${TEST_SOURCE_DIR}/check)
include_directories(${TEST_SOURCE_DIR}/check/src)

# gather all of source files. I have individually selected some files so that we don't run into compiler issues
# where some things are dependant on the ARM compiler.
# each variable here is separated into files that exist in different locations so we can build their individual
# absolute paths separately. We want to add each file individually as they are specific source files that we want
# to unit test.

# firmware/main
set(_MAIN "physics.c")
# firmware/main/util
set(_UTIL "quadratic.c" "log.c" "physbot.c" "util.c" "matrix.c")
# firmware/main/cvxgen
set(_CVXGEN "solver.c" "ldl.c" "matrix_support.c")
# firmware/main/primitives
set(_PRIMITIVES "move.c" "primitive.c")

# build the aboslute paths to each of the listed files
set_src_destinations("${_MAIN}" "${FIRMWARE_SOURCE_DIR}/main" "MAIN")
set_src_destinations("${_UTIL}" "${FIRMWARE_SOURCE_DIR}/main/util" "UTIL")
set_src_destinations("${_CVXGEN}" "${FIRMWARE_SOURCE_DIR}/main/cvxgen" "CVXGEN")
set_src_destinations("${_PRIMITIVES}" "${FIRMWARE_SOURCE_DIR}/main/primitives" "PRIMITIVES")

# glob the check and unit test source files
file(GLOB CHECK "${TEST_SOURCE_DIR}/check/src/*.c")
file(GLOB TEST_SOURCES "${TEST_SOURCE_DIR}/unit_tests/*.c")

# tell the compiler to ignore code wrapped in the FWTEST ifndef/endif. The code we are ignoring only works on the ARM
# compiler, but those files contain code that is required by other things in our source files
add_definitions("-DFWTEST")

# add the source files to the binary
add_executable(${BINARY_NAME}
        "${TEST_SOURCES}"
        "${TEST_SOURCE_DIR}/test.c"
        "${MAIN}"
        "${UTIL}"
        "${CVXGEN}"
        "${PRIMITIVES}"
        "${CHECK}")

# link against libraries
target_link_libraries(${BINARY_NAME}
        "m"
        "pthread"
        "rt")

# set the compile flags
target_compile_options(${BINARY_NAME}
        PUBLIC
        "-Wall"
        "-std=gnu99")
