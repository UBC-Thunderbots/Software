install_autoref() {
    autoref_commit=b30660b78728c3ce159de8ae096181a1ec52e9ba
    wget -N https://github.com/TIGERs-Mannheim/AutoReferee/archive/${autoref_commit}.zip -O /tmp/tbots_download_cache/autoReferee.zip
    unzip -q -o -d /tmp/tbots_download_cache/ /tmp/tbots_download_cache/autoReferee.zip

    /tmp/tbots_download_cache/AutoReferee-${autoref_commit}/./gradlew installDist -p /tmp/tbots_download_cache/AutoReferee-${autoref_commit} -Dorg.gradle.java.home=/opt/tbotspython/bin/jdk
    mv /tmp/tbots_download_cache/AutoReferee-${autoref_commit}/build/install/autoReferee /opt/tbotspython/
    rm -rf /tmp/tbots_download_cache/autoReferee.zip /tmp/tbots_download_cache/AutoReferee-${autoref_commit}
}

install_bazel() {
    download=https://github.com/bazelbuild/bazelisk/releases/download/v1.26.0/bazelisk-linux-arm64 

    if is_x86 $1; then
        download=https://github.com/bazelbuild/bazelisk/releases/download/v1.26.0/bazelisk-linux-amd64
    fi
    wget -nc $download -O /tmp/tbots_download_cache/bazel
    sudo mv /tmp/tbots_download_cache/bazel /usr/bin/bazel
    sudo chmod +x /usr/bin/bazel
}

install_clang_format() {
    ln -s /usr/bin/clang-format-14 /opt/tbotspython/bin/clang-format
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
    arch=arm64
    if is_x86 $1; then
        arch=amd64
    fi

    wget https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v3.16.1/ssl-game-controller_v3.16.1_linux_${arch} -O /tmp/tbots_download_cache/gamecontroller
    sudo mv /tmp/tbots_download_cache/gamecontroller /opt/tbotspython/gamecontroller
    sudo chmod +x /opt/tbotspython/gamecontroller
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
    mkdir -p /opt/tbotspython/cross_compile_headers
    wget -N https://www.python.org/ftp/python/3.12.0/Python-3.12.0.tar.xz -O /tmp/tbots_download_cache/python-3.12.0.tar.xz
    tar -xf /tmp/tbots_download_cache/python-3.12.0.tar.xz -C /tmp/tbots_download_cache/
    cd /tmp/tbots_download_cache/Python-3.12.0

    # The configuration is taken from the examples provided in https://docs.python.org/3.12/using/configure.html
    echo ac_cv_buggy_getaddrinfo=no > config.site-aarch64
    echo ac_cv_file__dev_ptmx=yes >> config.site-aarch64
    echo ac_cv_file__dev_ptc=no >> config.site-aarch64
    if is_x86 $1; then
        CONFIG_SITE=config.site-aarch64 ./configure \
            --build=x86_64-pc-linux-gnu \
            --host=aarch64-unknown-linux-gnu \
            -with-build-python=/usr/bin/python3.12 \
            --enable-optimizations \
            --prefix=/opt/tbotspython/cross_compile_headers > /dev/null
    else
        CONFIG_SITE=config.site-aarch64 ./configure \
            --build=aarch64-pc-linux-gnu \
            --host=aarch64-unknown-linux-gnu \
            -with-build-python=/usr/bin/python3.12 \
            --enable-optimizations \
            --prefix=/opt/tbotspython/cross_compile_headers > /dev/null
    fi
    make inclinstall -j$(nproc) > /dev/null

    cd -
    rm -rf /tmp/tbots_download_cache/Python-3.12.0
    rm -rf /tmp/tbots_download_cache/python-3.12.0.tar.xz
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
