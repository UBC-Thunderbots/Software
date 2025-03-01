install_autoref() {
    autoref_commit=b30660b78728c3ce159de8ae096181a1ec52e9ba
    sudo wget -N https://github.com/TIGERs-Mannheim/AutoReferee/archive/${autoref_commit}.zip -O /tmp/tbots_download_cache/autoReferee.zip
    unzip -q -o -d /tmp/tbots_download_cache/ /tmp/tbots_download_cache/autoReferee.zip

    /tmp/tbots_download_cache/AutoReferee-${autoref_commit}/./gradlew installDist -p /tmp/tbots_download_cache/AutoReferee-${autoref_commit} -Dorg.gradle.java.home=/opt/tbotspython/bin/jdk
    mv /tmp/tbots_download_cache/AutoReferee-${autoref_commit}/build/install/autoReferee /opt/tbotspython/
}

install_bazel() {
    download=https://github.com/bazelbuild/bazel/releases/download/5.4.0/bazel-5.4.0-linux-arm64

    if is_x86 $1; then
        download=https://github.com/bazelbuild/bazel/releases/download/5.4.0/bazel-5.4.0-linux-x86_64
    fi

    wget -nc $download -O /tmp/tbots_download_cache/bazel
    sudo mv /tmp/tbots_download_cache/bazel /usr/bin/bazel
    sudo chmod +x /usr/bin/bazel
}

install_clang_format() {
    arch=$1
    clang_format_file=clang+llvm-18.1.8-$arch-linux-gnu-ubuntu-18.04
    download=https://github.com/llvm/llvm-project/releases/download/llvmorg-18.1.8/$clang-format-file.tar.xz
    clang_format_path=/tmp/tbots_download_cache/$clang_format_file/bin/clang-format
    wget $download -O /tmp/tbots_download_cache/clang.tar.xz
    tar -xf /tmp/tbots_download_cache/clang.tar.xz -C /tmp/tbots_download_cache/
    sudo cp $clang_format_path /opt/tbotspython/bin/clang-format
}

install_cross_compiler() {
    file_name=aarch64-tbots-linux-gnu-for-aarch64
    if is_x86 $1; then
        file_name=aarch64-tbots-linux-gnu-for-x86
    fi
    full_file_name=$file_name.tar.xz
    wget https://raw.githubusercontent.com/UBC-Thunderbots/Software-External-Dependencies/refs/heads/tbots_compiler/toolchain/$full_file_name -O /tmp/tbots_download_cache/$full_file_name
    tar -xf /tmp/tbots_download_cache/$full_file_name -C /tmp/tbots_download_cache/
    sudo mv /tmp/tbots_download_cache/aarch64-tbots-linux-gnu /opt/tbotspython
    rm /tmp/tbots_download_cache/$full_file_name
}

install_gamecontroller () {
    # TODO(#3335): Whenever we deprecate Ubuntu 20.04, we can just grab the latest version of the SSL game controller
    # binary from the releases page. This is a workaround since the latest version of the game controller is compiled
    # with a newer GLIBC version than what is available on Ubuntu 20.04.
    go_arch=arm64
    if is_x86 $1; then
        go_arch=amd64
    fi
    sudo wget -N https://go.dev/dl/go1.23.0.linux-${go_arch}.tar.gz -O /tmp/tbots_download_cache/go.tar.gz
    tar -C /tmp/tbots_download_cache -xf /tmp/tbots_download_cache/go.tar.gz
    export PATH=$PATH:/tmp/tbots_download_cache/go/bin
    sudo wget -N https://github.com/RoboCup-SSL/ssl-game-controller/archive/refs/tags/v3.12.3.zip -O /tmp/tbots_download_cache/ssl-game-controller.zip
    unzip -q -o -d /tmp/tbots_download_cache/ /tmp/tbots_download_cache/ssl-game-controller.zip
    cd /tmp/tbots_download_cache/ssl-game-controller-3.12.3

    # Installing nvm
    wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
	export NVM_DIR="$HOME/.nvm"
	[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
	[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion
    nvm install 20

    make install
    go build cmd/ssl-game-controller/main.go
    sudo mv main /opt/tbotspython/gamecontroller
    sudo chmod +x /opt/tbotspython/gamecontroller

    cd -
    sudo rm -rf /tmp/tbots_download_cache/ssl-game-controller-3.12.3
    sudo rm -rf /tmp/tbots_download_cache/go
}

install_java () {
    java_home=""
    java_download=https://download.oracle.com/java/21/latest/jdk-21_linux-aarch64_bin.tar.gz
    if is_x86 $1; then
        java_download=https://download.oracle.com/java/21/latest/jdk-21_linux-x64_bin.tar.gz
    fi
    sudo wget -N $java_download -O /tmp/tbots_download_cache/jdk-21.tar.gz
    tar -xzf /tmp/tbots_download_cache/jdk-21.tar.gz -C /opt/tbotspython/
    mv /opt/tbotspython/jdk-21* /opt/tbotspython/bin/jdk
}

is_x86() {
    if [[ $1 == "x86_64" ]]; then
        return 0
    else
        return 1
    fi
}

print_status_msg () {
   echo "================================================================"
   echo $1
   echo "================================================================"
}
