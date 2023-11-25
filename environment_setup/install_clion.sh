#!/bin/bash

echo "================================================================"
echo "Installing CLion"
echo "================================================================"

clion_version_year="2021"
clion_version_major="2"
clion_version_minor="4"
clion_version="${clion_version_year}.${clion_version_major}.${clion_version_minor}"
clion_executable_path="/usr/local/bin/clion"


# Check the correct clion version is installed
if [ -e "/opt/clion-${clion_version}/bin/clion.sh" ] && [ -e ${clion_executable_path}-${clion_version} ]
then
        echo "================================================================"
        echo "CLion is already installed"
        echo "================================================================"
else
    # Download clion
    wget -O /tmp/CLion-${clion_version}.tar.gz "https://download.jetbrains.com/cpp/CLion-${clion_version}.tar.gz"

    # Unzip and symlink to usr location
    sudo tar xfz /tmp/CLion-${clion_version}.tar.gz -C /opt
    
    # Install clion desktop entry
    wget -O ~/.local/share/applications/jetbrains-clion.desktop "https://raw.githubusercontent.com/pld-linux/clion/master/clion.desktop"
    echo "Icon=/opt/clion-${clion_version}/bin/clion.png" >> ~/.local/share/applications/jetbrains-clion.desktop
fi

echo "================================================================"
echo "Symlinking CLion"
echo "================================================================"
# Symlink clion binary to a location in the PATH
# We do this outside the above `if` because this gives us the ability to 
# change clion versions without re-downloading
sudo ln -s -f /opt/clion-${clion_version}/bin/clion.sh ${clion_executable_path}-${clion_version}
sudo ln -s -f ${clion_executable_path}-${clion_version} $clion_executable_path

echo "================================================================"
echo "Installing Bazel Plugin"
echo "================================================================"

clion_plugin_dir="${HOME}/.CLion${clion_version_year}.${clion_version_major}/config/plugins"
bazel_plugin_version_year="2022"
bazel_plugin_version_major="03"
bazel_plugin_version_minor="22"
bazel_plugin_version="v${bazel_plugin_version_year}.${bazel_plugin_version_major}.${bazel_plugin_version_minor}"

if [ -d "${clion_plugin_dir}/clwb" ]
then
    echo "================================================================"
    echo "Bazel Plugin Already Installed"
    echo "To force reinstallation please remove ${clion_plugin_dir}/clwb".
    echo "================================================================"
else
    # Download bazel plugin
    wget -O /tmp/bazelbuild-${bazel_plugin_version}.tar.gz "https://github.com/bazelbuild/intellij/archive/${bazel_plugin_version}.tar.gz"

    # Unpack and build the plugin from the source
    mkdir -p /tmp/bazelbuild-${bazel_plugin_version} 
    tar xfz /tmp/bazelbuild-${bazel_plugin_version}.tar.gz -C /tmp/bazelbuild-${bazel_plugin_version}
    cd /tmp/bazelbuild-${bazel_plugin_version}/intellij* || exit
    bazel build //clwb:clwb_bazel_zip --define=ij_product=clion-${clion_version_year}.${clion_version_major}

    # Copy the compiled plugin to the CLion directory
    mkdir -p "$clion_plugin_dir"
    unzip bazel-bin/clwb/clwb_bazel.zip -d "$clion_plugin_dir"

    # Cleanup
    rm -rf /tmp/bazelbuild
fi


echo "================================================================"
echo "Done"
echo "================================================================"
