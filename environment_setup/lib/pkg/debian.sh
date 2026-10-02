# Debian / Ubuntu package backend.

pkg_prepare_repositories() {
  # software-properties-common provides add-apt-repository
  as_root apt-get install -y software-properties-common
  as_root add-apt-repository -y ppa:ubuntu-toolchain-r/test
  as_root add-apt-repository -y ppa:deadsnakes/ppa
  as_root apt-get update
}

pkg_install_names() {
  DEBIAN_FRONTEND=noninteractive as_root apt-get install -y "$@"
}

# Maps a logical dependency to the Ubuntu packages that provide it.
pkg_resolve() {
  case "$1" in
    cmake)                echo "cmake" ;;
    clang-format)         echo "clang-format-14" ;;
    codespell)            echo "codespell" ;;
    curl)                 echo "curl" ;;
    git)                  echo "git" ;;
    jdk)                  echo "default-jdk" ;;
    gcc)                  echo "gcc-10" ;;
    g++)                  echo "g++-10" ;;
    kcachegrind)          echo "kcachegrind" ;;
    eigen)                echo "libeigen3-dev" ;;
    sqlite3-dev)          echo "libsqlite3-dev" ;;
    ffi-dev)              echo "libffi-dev" ;;
    ssl-dev)              echo "libssl-dev" ;;
    openssl)              echo "openssl" ;;
    sshpass)              echo "sshpass" ;;
    unzip)                echo "unzip" ;;
    valgrind)             echo "valgrind" ;;
    xvfb)                 echo "xvfb" ;;
    libstdcxx-dbg)        echo "libstdc++6-9-dbg" ;;
    python)               echo "python3.12" ;;
    python-dev)           echo "python3.12-dev" ;;
    python-venv)          echo "python3.12-venv" ;;
    python-pip)           echo "python3-pip" ;;
    python-yaml)          echo "python3-yaml" ;;
    udev-dev)             echo "libudev-dev" ;;
    usb-dev)              echo "libusb-1.0-0-dev" ;;
    xcb-cursor)           echo "libxcb-cursor0" ;;
    pyqt)                 echo "python3-pyqt6 pyqt6-dev-tools python3-pyqt6.qtsvg" ;;
    libopengl0)           echo "libopengl0" ;;
    *)                    return 1 ;;
  esac
}
