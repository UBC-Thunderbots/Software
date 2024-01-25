# Save the parent dir of this so we can always run commands relative to the
# location of this script, no matter where it is called from. This
# helps prevent bugs and odd behaviour if this script is run through a symlink
# or from a different directory.
CURR_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")
cd "$CURR_DIR" || exit

# Install capstone-next
wget -nc https://github.com/capstone-engine/capstone/archive/refs/tags/5.0.1.zip -O /tmp/capstone.zip
cd /tmp/
unzip capstone.zip
cd capstone-5.0.1/
./make.sh
sudo ./make.sh install
cd "$CURR_DIR"

# Build Tracy
wget -nc https://github.com/wolfpld/tracy/archive/refs/tags/v0.10.tar.gz -O /tmp/tracy.tar.gz
cd /tmp/
tar -xvf /tmp/tracy.tar.gz
cd /tmp/tracy-0.10/profiler/build/unix/
make LEGACY=1
cp /tmp/tracy-0.10/profiler/build/unix/Tracy-release /opt/tbotspython/bin
