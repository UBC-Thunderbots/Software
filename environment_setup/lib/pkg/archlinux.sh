# Arch Linux package backend.
#
# This backend exists to demonstrate the extension point: it is a mapping from
# logical dependencies to pacman packages, and reuses the portable install
# routines in lib/providers/default.sh for everything else.

pkg_prepare_repositories() {
  as_root pacman -Syu --noconfirm
}

pkg_install_names() {
  as_root pacman -S --needed --noconfirm "$@"
}

# Maps a logical dependency to the Arch packages that provide it. Logical
# dependencies that Arch does not ship an equivalent for are simply omitted and
# will be reported as skipped by pkg_install.
pkg_resolve() {
  case "$1" in
    cmake)                echo "cmake" ;;
    clang-format)         echo "clang" ;;
    codespell)            echo "codespell" ;;
    curl)                 echo "curl" ;;
    git)                  echo "git" ;;
    jdk)                  echo "jdk" ;;
    gcc)                  echo "gcc" ;;
    g++)                  echo "g++" ;;
    kcachegrind)          echo "kcachegrind" ;;
    eigen)                echo "eigen" ;;
    sqlite3-dev)          echo "sqlite" ;;
    ffi-dev)              echo "libffi" ;;
    ssl-dev)              echo "openssl" ;;
    openssl)              echo "openssl" ;;
    sshpass)              echo "sshpass" ;;
    unzip)                echo "unzip" ;;
    valgrind)             echo "valgrind" ;;
    xvfb)                 echo "xvfb" ;;
    python)               echo "python3.12" ;;
    python-dev)           echo "python3.12" ;;
    python-venv)          echo "python3.12" ;;
    python-pip)           echo "python-pip" ;;
    python-yaml)          echo "python-yaml" ;;
    udev-dev)             echo "libudev" ;;
    usb-dev)              echo "libusb" ;;
    xcb-cursor)           echo "libxcb-cursor" ;;
    pyqt)                 echo "pyqt6" ;;
    libopengl0)           echo "libglvnd" ;;
    node)                 echo "nodejs" ;;
    go)                   echo "go" ;;
    *)                    return 1 ;;
  esac
}
