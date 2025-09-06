install_autoref() {
    autoref_commit=b30660b78728c3ce159de8ae096181a1ec52e9ba
    wget -N https://github.com/TIGERs-Mannheim/AutoReferee/archive/${autoref_commit}.zip -O /tmp/tbots_download_cache/autoReferee.zip
    unzip -q -o -d /tmp/tbots_download_cache/ /tmp/tbots_download_cache/autoReferee.zip

    /tmp/tbots_download_cache/AutoReferee-${autoref_commit}/./gradlew installDist -p /tmp/tbots_download_cache/AutoReferee-${autoref_commit} -Dorg.gradle.java.home=/opt/tbotspython/bin/jdk
    mv /tmp/tbots_download_cache/AutoReferee-${autoref_commit}/build/install/autoReferee /opt/tbotspython/
    rm -rf /tmp/tbots_download_cache/autoReferee.zip /tmp/tbots_download_cache/AutoReferee-${autoref_commit}
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
    download=https://github.com/llvm/llvm-project/releases/download/llvmorg-19.1.7/clang+llvm-19.1.7-aarch64-linux-gnu.tar.xz
    clang_folder=clang+llvm-19.1.7-aarch64-linux-gnu

    if is_x86 $1; then
        download=https://github.com/llvm/llvm-project/releases/download/llvmorg-19.1.7/LLVM-19.1.7-Linux-X64.tar.xz
        clang_folder=LLVM-19.1.7-Linux-X64
    fi

    wget $download -O /tmp/tbots_download_cache/clang.tar.xz

    # Temporarily need more space to extract the clang tarball
    mkdir -p ~/.tbots
    tar -xf /tmp/tbots_download_cache/clang.tar.xz -C ~/.tbots/

    clang_format_path=~/.tbots/$clang_folder/bin/clang-format
    sudo cp $clang_format_path /opt/tbotspython/bin/clang-format
    rm -rf ~/.tbots
}

install_cross_compiler() {
    file_name=aarch64-tbots-linux-gnu-for-aarch64
    if is_x86 $1; then
        file_name=aarch64-tbots-linux-gnu-for-x86
    fi
    full_file_name=$file_name.tar.xz
    wget https://raw.githubusercontent.com/UBC-Thunderbots/Software-External-Dependencies/refs/heads/main/toolchain/$full_file_name -O /tmp/tbots_download_cache/$full_file_name
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
    sudo rm -rf /tmp/tbots_download_cache/ssl-game-controller-3.12.3 /tmp/tbots_download_cache/go /tmp/tbots_download_cache/go.tar.gz /tmp/tbots_download_cache/ssl-game-controller.zip
}

install_java () {
    java_home=""
    java_download=https://download.oracle.com/java/21/latest/jdk-21_linux-aarch64_bin.tar.gz
    if is_x86 $1; then
        java_download=https://download.oracle.com/java/21/latest/jdk-21_linux-x64_bin.tar.gz
    fi
    wget -N $java_download -O /tmp/tbots_download_cache/jdk-21.tar.gz
    tar -xzf /tmp/tbots_download_cache/jdk-21.tar.gz -C /opt/tbotspython/
    mv /opt/tbotspython/jdk-21* /opt/tbotspython/bin/jdk
    rm /tmp/tbots_download_cache/jdk-21.tar.gz
}

install_python_dev_cross_compile_headers() {
    if is_x86 $1; then
        mkdir -p /opt/tbotspython/cross_compile_headers
        wget -N https://www.python.org/ftp/python/3.12.0/Python-3.12.0.tar.xz -O /tmp/tbots_download_cache/python-3.12.0.tar.xz
        tar -xf /tmp/tbots_download_cache/python-3.12.0.tar.xz -C /tmp/tbots_download_cache/
        cd /tmp/tbots_download_cache/Python-3.12.0

        # The configuration is taken from the examples provided in https://docs.python.org/3.12/using/configure.html
        echo ac_cv_buggy_getaddrinfo=no > config.site-aarch64
        echo ac_cv_file__dev_ptmx=yes >> config.site-aarch64
        echo ac_cv_file__dev_ptc=no >> config.site-aarch64
        CONFIG_SITE=config.site-aarch64 ./configure \
            --build=x86_64-pc-linux-gnu \
            --host=aarch64-unknown-linux-gnu \
            -with-build-python=/usr/bin/python3.12 \
            --enable-optimizations \
            --prefix=/opt/tbotspython/cross_compile_headers > /dev/null
        make inclinstall -j$(nproc) > /dev/null

        cd -
        rm -rf /tmp/tbots_download_cache/Python-3.12.0
        rm -rf /tmp/tbots_download_cache/python-3.12.0.tar.xz
    fi
}

install_stm32_cross_compiler() {
    download_link=https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-x86_64-aarch64_be-none-linux-gnu.tar.xz

    if is_x86 $1; then
        download_link=https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi.tar.xz
    fi

    wget -N $download_link -O /tmp/tbots_download_cache/arm-gnu-toolchain.tar.xz
    tar -xf /tmp/tbots_download_cache/arm-gnu-toolchain.tar.xz -C /tmp/tbots_download_cache/
    sudo mv /tmp/tbots_download_cache/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi /opt/tbotspython/arm-none-eabi-gcc
    rm /tmp/tbots_download_cache/arm-gnu-toolchain.tar.xz
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
