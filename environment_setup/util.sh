install_autoref () {
    sudo wget -N https://github.com/TIGERs-Mannheim/AutoReferee/archive/refs/heads/autoref-ci.zip -O /tmp/autoref-ci.zip
    unzip -q -o -d /tmp/ /tmp/autoref-ci.zip
    touch /tmp/AutoReferee-autoref-ci/.git # a hacky way to make gradle happy when it tries to find a dependency

    java_home=/opt/tbotspython/jdk-17.0.12/
    mirror=""
    if is_x86 $1; then
        java_home=/usr/lib/jvm/jdk-17.0.12-oracle-x64/
        mirror="https://github.com/ubc-thunderbots/autoreferee/releases/download/autoref-ci/autoreferee.tar.gz"
    fi

    ln -s /opt/tbotspython/bin/jdk $java_home

    if ! /tmp/AutoReferee-autoref-ci/./gradlew installDist -p /tmp/AutoReferee-autoref-ci/ -Dorg.gradle.java.home=$java_home; then
        print_status_msg "Building TIGERS AutoRef failed. Downloading mirror"
    
        wget $mirror -o /tmp/autoreferee.tar.gz
        tar -xzf /tmp/autoReferee.tar.gz -C /opt/tbotspython/
    else
        cp -r /tmp/AutoReferee-autoref-ci/build/install/autoReferee/ /opt/tbotspython/autoReferee
    fi
}

install_bazel() {
    download=https://github.com/bazelbuild/bazel/releases/download/5.4.0/bazel-5.4.0-linux-arm64

    if is_x86 $1; then
        download=https://github.com/bazelbuild/bazel/releases/download/5.4.0/bazel-5.4.0-linux-x86_64
    fi

    wget -nc $download -O /tmp/bazel
    sudo mv /tmp/bazel /usr/bin/
    sudo chmod +x /usr/bin/bazel
}

install_gamecontroller () {
    if is_x86 $1; then
        sudo wget -N https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v3.12.3/ssl-game-controller_v3.12.3_linux_amd64 -O /opt/tbotspython/gamecontroller
    else
        sudo wget -N https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v3.12.3/ssl-game-controller_v3.12.3_linux_arm64 -O /opt/tbotspython/gamecontroller
    fi
    sudo chmod +x /opt/tbotspython/gamecontroller
}

install_java () {
    if is_x86 $1; then
        sudo wget -N https://download.oracle.com/java/17/archive/jdk-17.0.12_linux-x64_bin.deb -O /tmp/jdk-17.0.12.deb
        sudo apt install /tmp/./jdk-17.0.12.deb
        return
    fi
    sudo wget https://download.oracle.com/java/17/latest/jdk-17_linux-aarch64_bin.tar.gz -O /opt/jdk-17.0.12.tar.gz
    tar -xvzf jdk-17.0.12.tar.gz -C /opt/tbotspython/
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
