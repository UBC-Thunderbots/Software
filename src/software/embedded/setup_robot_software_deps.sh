#!/bin/bash

set -ex

host_software_packages=(
    device-tree-compiler
    curl
    python3
    python3-venv
    python3-pip
    libgpiod-dev
    libssl-dev
    libffi-dev
    zlib1g-dev
    libbz2-dev
    libreadline-dev
    libsqlite3-dev
    libncursesw5-dev
    tk-dev
    libgdbm-dev
    libc6-dev
    liblzma-dev
)

# Install packages
sudo apt-get update
sudo apt-get install "${host_software_packages[@]}" -y

# Install platformio udev rules
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules

# Install Python 3.12
mkdir -p /tmp/tbots_download_cache
wget -N https://www.python.org/ftp/python/3.12.0/Python-3.12.0.tar.xz -O /tmp/tbots_download_cache/python-3.12.0.tar.xz
tar -xf /tmp/tbots_download_cache/python-3.12.0.tar.xz -C /tmp/tbots_download_cache/
cd /tmp/tbots_download_cache/Python-3.12.0
./configure --enable-optimizations
make -j$(nproc)
sudo make altinstall

# Remove outdated /opt/tbotspython
sudo rm -rf /opt/tbotspython

sudo mkdir -p /opt/tbotspython
sudo chown -R $USER:$USER /opt/tbotspython

if ! sudo /usr/local/bin/python3.12 -m venv /opt/tbotspython ; then
    echo "Error: Installing Python 3.12 failed"
    exit 1
fi

# install PlatformIO to global environment
curl -fsSL -o /tmp/tbots_download_cache/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
if ! /usr/local/bin/python3.12 /tmp/tbots_download_cache/get-platformio.py; then
    echo "Error: Installing PlatformIO failed"
    exit 1
fi

ln -s $HOME/.platformio/penv/bin/platformio /opt/tbotspython/bin/platformio

# Pre-install everything PlatformIO needs to flash the powerboard so that
# flashing works even when the Pi has no internet access.
#
# deploy_powerboard.yml flashes with `platformio run -t nobuild -t upload`.
# `pio run` re-resolves and installs the project environment's packages on every
# invocation. The prebuilt firmware and project libraries travel inside
# powerloop.tar.gz, but the espressif32 platform, the xtensa toolchain, the
# esptool uploader and the Arduino framework live in the *global* ~/.platformio
# cache and are NOT shipped. framework-arduinoespressif32 is an *optional*
# espressif32 package, so a bare `platform install espressif32` does not fetch
# it and the first offline flash would fail trying to download it.
#
# Warm the global cache here (setup runs with verified internet) by resolving the
# exact same packages `pio run` needs from a platformio.ini that mirrors the
# powerloop project environment (keep in sync with src/software/power/BUILD).
# Installing via a project dir (not `-p`) is what pulls the optional framework.
POWERLOOP_PIO_WARMUP="$(mktemp -d)"
cat > "$POWERLOOP_PIO_WARMUP/platformio.ini" <<'EOF'
[env:pico32]
platform = espressif32
board = pico32
framework = arduino
EOF
/opt/tbotspython/bin/platformio pkg install -d "$POWERLOOP_PIO_WARMUP"
rm -rf "$POWERLOOP_PIO_WARMUP"

# Programmatically enable serial communication for UART
sudo raspi-config nonint do_serial_hw 0
sudo raspi-config nonint do_serial_cons 1
