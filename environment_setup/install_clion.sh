#!/bin/bash

echo "================================================================"
echo "Installing CLion"
echo "================================================================"

clion_version="2019.2.2"
clion_executable_path="/usr/local/bin/clion"

# Check the correct clion version is installed
if [ -e "/opt/clion-${clion_version}/bin/clion.sh" ] && [ -e ${clion_executable_path}-${clion_version} ]
then
        echo "================================================================"
        echo "CLion is already installed"
        echo "================================================================"
else
	# Download clion
        wget -O /tmp/CLion-${clion_version}.tar.gz "https://download-cf.jetbrains.com/cpp/CLion-${clion_version}.tar.gz"

	# Unzip and symlink to usr location
        sudo tar xfz /tmp/CLion-${clion_version}.tar.gz -C /opt
        sudo ln -s /opt/clion-${clion_version}/bin/clion.sh ${clion_executable_path}-${clion_version}
        sudo ln -s ${clion_executable_path}-${clion_version} $clion_executable_path
fi

echo "================================================================"
echo "Installing Bazel Plugin"
echo "================================================================"

clion_plugin_dir="${HOME}/.CLion2019.2/config/plugins"
clion_major_version="2019.2"
bazel_plugin_version="v2019.09.16"

if [ -d "${clion_plugin_dir}/clwb" ]
then
	echo "================================================================"
	echo "Bazel Plugin Already Installed"
	echo "================================================================"
else
	# Download bazel plugin
	wget -O /tmp/bazelbuild.tar.gz "https://github.com/bazelbuild/intellij/archive/${bazel_plugin_version}.tar.gz"

	# Unpack and build the plugin from the source
	mkdir -p /tmp/bazelbuild
	tar xfz /tmp/bazelbuild.tar.gz -C /tmp/bazelbuild
	cd /tmp/bazelbuild/intellij*
	bazel build //clwb:clwb_bazel_zip --define=ij_product=clion-${clion_major_version}

	# Copy the compiled plugin to the CLion directory
	mkdir -p $clion_plugin_dir
	unzip bazel-bin/clwb/clwb_bazel.zip -d $clion_plugin_dir

	# Cleanup
	rm -rf /tmp/bazelbuild
fi


echo "================================================================"
echo "Done"
echo "================================================================"
