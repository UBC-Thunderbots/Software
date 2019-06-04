#!/usr/bin/env bash

dir="$1"
target="$2"
options="$3"

# show the help when the user types
# ./cmake.sh --help
if [[ $dir == "--help" ]]; then
    echo "To use the cmake bash script, type the following:"
    echo "    ./cmake.sh directory target"
    echo
    echo "The directory is one of:"
    printf "    software firmware/main\n    firmware/dongle\n    firmware/test\n    tools/fwsim\n"
    echo
    echo "software targets are one of: "
    printf "    ai\n    mrftest\n    software_test\n    buildid\n    getcore\n    hall2phase\n    log\n    mrfcap\n    nulltest\n    sdutil\n"
    echo
    echo "firmware/main targets are one of: "
    printf "    robot_firmware.elf\n    robot_firmware.dfuse\n    robot_firmware.dfu\n"
    echo
    echo "firmware/dongle targets are one of: "
    printf "    dongle_firmware.elf\n    dongle_firmware.dfu\n"
    echo
    echo "For firmware/test the target is fw_test"
    echo "For tools/fwsim the target is sim"
    echo
    printf "Run\n    ./cmake.sh directory all\nto build all of the executables in that directory \n"
    echo
    printf "Example usage:\n    ./cmake.sh software ai\n"
    echo
    printf "Run\n    ./cmake.sh --clean\nto clean the build files"
    echo
# clean the build folder
elif [[ $dir == "--clean" ]]; then
    rm -r cmake-build-debug
# make sure that an actual directory and target are supplied
elif [[ ${#dir} == 0 || ${#target} == 0 ]]; then
    echo "Invalid number of arguments supplied"
    echo "Type ./cmake.sh --help for instructions on how to use this script"
# build the code for the given target in the given directory
else
    cmake -DCMAKE_BUILD_TYPE=Debug -Bcmake-build-debug -H.
    cmake --build cmake-build-debug --target $target -- -C $dir $options
fi
