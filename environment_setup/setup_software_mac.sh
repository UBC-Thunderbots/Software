#!/bin/bash
set -x
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# UBC Thunderbots macOS Software Setup
#
# This script will install all required libraries and dependencies to build
# and run the Thunderbots codebase on macOS, including the AI and unit tests.
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Save the parent dir of this script
CURR_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
cd "$CURR_DIR" || exit

source util.sh

arch=$(uname -m)
print_status_msg "Detected architecture: ${arch}"

# Check for Homebrew and install if missing
if ! command -v brew &>/dev/null; then
    print_status_msg "Installing Homebrew..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

    # Configure Homebrew in current shell
    if [ "$arch" = "arm64" ]; then
        eval "$(/opt/homebrew/bin/brew shellenv)"
    else
        eval "$(/usr/local/bin/brew shellenv)"
    fi
fi

print_status_msg "Installing Utilities and Dependencies"

# Update Homebrew
brew update

# Install required packages
host_software_packages=(
    cmake@4
    python@3.12
    bazelisk
    openjdk@21
    pyqt@6
    qt@6
    node@20
    go@1.24
    clang-format@20
)

for pkg in "${host_software_packages[@]}"; do
    if ! brew list "$pkg" &>/dev/null; then
        print_status_msg "Installing $pkg..."
        brew install "$pkg"
    else
        print_status_msg "$pkg already installed, skipping..."
    fi
done

# Set up cache
mkdir /tmp/tbots_download_cache

# Set up Python
print_status_msg "Setting Up Python Environment"

# Create virtual environment
sudo python3.12 -m venv /opt/tbotspython
chmod
source /opt/tbotspython/bin/activate

# Install Python dependencies
sudo pip install --upgrade pip
sudo pip install -r macos_requirements.txt

print_status_msg "Done Setting Up Python Environment"

print_status_msg "Fetching game controller"
install_gamecontroller_macos

print_status_msg "Setting up TIGERS AutoRef"
install_java_macos
install_autoref_macos
sudo chmod +x "$CURR_DIR/../src/software/autoref/run_autoref.sh"
sudo cp "$CURR_DIR/../src/software/autoref/DIV_B.txt" "/opt/tbotspython/autoReferee/config/geometry/DIV_B.txt"
print_status_msg "Finished setting up AutoRef"

print_status_msg "Setting up cross compiler for robot software"
install_cross_compiler_mac
print_status_msg "Done setting up cross compiler for robot software"

print_status_msg "Setting Up Python Development Headers"
install_python_toolchain_headers_macos
print_status_msg "Done Setting Up Python Development Headers"

print_status_msg "Granting Permissions to /opt/tbotspython"
sudo chown -R $(id -u):$(id -g) /opt/tbotspython
print_status_msg "Done Granting Permissions to /opt/tbotspython"

print_status_msg "Done Environment Configuration"

print_status_msg "Software Setup Complete"
print_status_msg "Note: Some changes require a new terminal session to take effect"

