#!/bin/bash

# Any failure should terminate this script
set -e

# download version 6.2.0 (change the following three variables to change versions)
CUBE_VERSION="6.2.0"

# The original zip file can be found here: https://www.st.com/en/development-tools/stm32cubemx.html
# It was installed locally and opened once (to get updates) and then zipped up (See install_cubemx_from_source.sh)
# It was split using `split --bytes=49M zip_file`

CURR_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")
CUBEMX_TMP_DIR="/tmp/cubemx"
CUBEMX_INSTALL_DIR="/opt/STM32CubeMX_$CUBE_VERSION"

if [ -d "$CUBEMX_INSTALL_DIR" ]; then
    echo "================================================================"
    echo " CubeMX $CUBE_VERSION already installed "
    echo "================================================================"
else
    echo "================================================================"
    echo "Installing CubeMX $CUBE_VERSION" 
    echo "================================================================"

    # setup a clean directory to install cubemx in
    rm -rf $CUBEMX_TMP_DIR
    mkdir -p $CUBEMX_TMP_DIR
    cd $CUBEMX_TMP_DIR

    cp $CURR_DIR/stm32cubemx-6.2.0/* .
    cat x* > STM32CubeMX_$CUBE_VERSION.zip

    if ! unzip STM32CubeMX_$CUBE_VERSION.zip ; then
        echo "##############################################################"
        echo "Error: Installing CubeMX failed"
        echo "Could not unzip STM32CubeMX_$CUBE_VERSION.zip"
        echo "##############################################################"
        exit 1
    fi

    sudo cp -r STM32CubeMX_$CUBE_VERSION $CUBEMX_INSTALL_DIR
    
    sudo chmod 777 -R /opt/STM32CubeMX_$CUBE_VERSION

    cd $CUBEMX_INSTALL_DIR

    sudo chmod +x cuberunner.sh
    sudo ln -sfn /opt/STM32CubeMX_$CUBE_VERSION/cuberunner.sh /usr/local/bin/STM32CubeMX 

    if [[ $? == 0 ]] ; then
        echo "================================================================"
        echo "Done Installing CubeMX to $CUBEMX_INSTALL_DIR"
        echo "NOTE: if you are upgrading versions, you can delete older versions in the /opt directory"
        echo "================================================================"
    else
        echo "##############################################################"
        echo "Error: Installing CubeMX failed"
        echo "##############################################################"
        exit 1
    fi
fi
