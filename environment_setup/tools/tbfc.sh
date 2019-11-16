#!/usr/bin/env bash

# Define TBOTS_DIR to be the root of the Software repo in your .bashrc/.bash_profile
ProgName=tb
  
sub_help(){
    echo "Usage: $ProgName <subcommand> [options]\n"
    echo "Subcommands:"
    echo "    build {ai, fw} [opts]"
    echo "    run  {ai, fw} [opts] "
    echo "    macos {pfw, ufw, ai} [opts] ### EXPERIMENTAL ###"
    echo ""
    echo "For help with each subcommand run:"
    echo "$ProgName <subcommand> -h|--help"
    echo ""
}
  
sub_build(){
    echo "Running 'build' command."
    sub=$1
    shift
    echo $@
    case $sub in
        "fw")
            bazel build //firmware_new/... --cpu=stm32h7;
            ;;
        "ai")
            bazel build //software/...;
            ;;
        *)
    esac

}
  
sub_run(){
    sub=$1
    shift
    echo $@
    case $sub in
        "fw")
            bazel run --cpu=stm32h7 --compilation_mode=dbg //firmware_new/tools:debug_firmware_on_arm_board;
            ;;
        "ai")
            bazel run //software/full_system $@;
            ;;
        *)
    esac
}

sub_macos(){
    sub=$1
    shift
    echo $@
    case $sub in
        "pfw")
            # Patch workspace 
            git apply $TBOTS_DIR/environment_setup/tools/macos_fw.patch
            ;;
        "ufw")
            # Unpatch 
            git apply -R $TBOTS_DIR/environment_setup/tools/macos_fw.patch
            ;;
        "ai")
            echo "Sorry, no support for ai yet :("
            exit 1
            ;;
        *)
    esac
}
  
function tb() {
    subcommand=$1
    case $subcommand in
        "-h" | "--help")
            sub_help
            ;;
        "")
            cd $TBOTS_DIR;
            ;;
        *)
            shift
            cd $TBOTS_DIR/src;
            sub_${subcommand} $@
            if [ $? = 127 ]; then
                echo "Error: '$subcommand' is not a known subcommand." >&2
                echo "       Run '$ProgName --help' for a list of known subcommands." >&2
                exit 1
            fi
            ;;
    esac
}