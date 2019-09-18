#!/bin/bash

echo "================================================================"
echo "Installing CLion"
echo "================================================================"

clion_version="2019.2.2"
usr_location="/usr/local/bin/clion"

if [ -e ${usr_location} ]
then
        echo "================================================================"
        echo "CLion is already installed"
        echo "================================================================"
else
	# Download clion
        wget -O ~/Downloads/CLion-${clion_version}.tar.gz "https://download-cf.jetbrains.com/cpp/CLion-${clion_version}.tar.gz"

	# Unzip and symlink to usr location
        sudo mkdir -p /opt/clion
        sudo tar xfz ~/Downloads/CLion-${clion_version}.tar.gz -C /opt/clion
        sudo ln -s /opt/clion/clion-${clion_version}/bin/clion.sh /usr/local/bin/clion
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
	wget -O ~/Downloads/bazelbuild.tar.gz "https://github.com/bazelbuild/intellij/archive/${bazel_plugin_version}.tar.gz"

	# Unpack and build the plugin from the source
	mkdir -p bazelbuild
	tar xfz ~/Downloads/bazelbuild.tar.gz -C bazelbuild
	cd bazelbuild/intellij*
	sudo bazel build //clwb:clwb_bazel_zip --define=ij_product=clion-${clion_major_version}

	# Copy the compiled plugin to the CLion directory
	mkdir -p ~/.CLion${clion_major_version}/config/plugins
	unzip bazel-bin/clwb/clwb_bazel.zip -d $clion_plugin_dir

	# Cleanup
	rm -rf ../../bazelbuild
fi


echo "================================================================"
echo "Done"
echo "================================================================"
