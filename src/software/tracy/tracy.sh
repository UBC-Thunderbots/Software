if ! [ -e "/opt/tbotspython/bin/Tracy-release" ]; then
    echo "Error: Please run ./install_tracy.sh, Tracy binary not installed"
    exit 1
fi

/opt/tbotspython/bin/Tracy-release
