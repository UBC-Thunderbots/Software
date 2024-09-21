install_autoref() {
    autoref_commit=b30660b78728c3ce159de8ae096181a1ec52e9ba
    sudo wget -N https://github.com/TIGERs-Mannheim/AutoReferee/archive/${autoref_commit}.zip -O /tmp/autoReferee.zip
    unzip -q -o -d /tmp/ /tmp/autoReferee.zip

    /tmp/AutoReferee-${autoref_commit}/./gradlew installDist -p /tmp/AutoReferee-${autoref_commit} -Dorg.gradle.java.home=/opt/tbotspython/bin/jdk
    cp -r /tmp/AutoReferee-${autoref_commit}/build/install/autoReferee /opt/tbotspython/
}

install_bazel() {
    download=https://github.com/bazelbuild/bazel/releases/download/5.4.0/bazel-5.4.0-linux-arm64

    if is_x86 $1; then
        download=https://github.com/bazelbuild/bazel/releases/download/5.4.0/bazel-5.4.0-linux-x86_64
    fi

    wget -nc $download -O /tmp/bazel
    sudo mv /tmp/bazel /usr/bin/bazel
    sudo chmod +x /usr/bin/bazel
}

install_gamecontroller () {
    go_arch=arm64
    if is_x86 $1; then
        go_arch=amd64
    fi
    sudo wget -N https://go.dev/dl/go1.23.0.linux-${go_arch}.tar.gz -O /tmp/go.tar.gz
    tar -C /tmp -xf /tmp/go.tar.gz
    export PATH=$PATH:/tmp/go/bin
    sudo wget -N https://github.com/RoboCup-SSL/ssl-game-controller/archive/refs/tags/v3.12.3.zip -O /tmp/ssl-game-controller.zip
    unzip -q -o -d /tmp/ /tmp/ssl-game-controller.zip
    cd /tmp/ssl-game-controller-3.12.3
    wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
    make install
    go build cmd/ssl-game-controller/main.go
    sudo mv main /opt/tbotspython/gamecontroller
    sudo chmod +x /opt/tbotspython/gamecontroller
}

install_java () {
    java_home=""
    java_download=https://download.oracle.com/java/21/latest/jdk-21_linux-aarch64_bin.tar.gz
    if is_x86 $1; then
        java_download=https://download.oracle.com/java/21/latest/jdk-21_linux-x64_bin.tar.gz
    fi
    sudo wget -N $java_download -O /tmp/jdk-21.tar.gz
    tar -xzf /tmp/jdk-21.tar.gz -C /opt/tbotspython/
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
