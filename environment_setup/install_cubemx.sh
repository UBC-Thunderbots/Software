#!/bin/bash

# Any failure should terminate this script
set -e

# download version 6.2.0 (change the following three variables to change versions)
CUBE_VERSION="6.2.0"
CUBE_ZIP_FILENAME="en.stm32cubemx-lin_v6-2-0.zip"

# The original zip file can be found here: https://www.st.com/en/development-tools/stm32cubemx.html
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

    # create auto install xml
    AUTO_INSTALL_XML="
        <?xml version='1.0' encoding='UTF-8' standalone='no'?>
        <AutomatedInstallation langpack='eng'>
            <com.st.microxplorer.install.MXHTMLHelloPanel id='readme'/>
            <com.st.microxplorer.install.MXLicensePanel id='licence.panel'/>
            <com.st.microxplorer.install.MXTargetPanel id='target.panel'>
            <installpath>$CUBEMX_INSTALL_DIR</installpath>
            </com.st.microxplorer.install.MXTargetPanel>
            <com.st.microxplorer.install.MXShortcutPanel id='shortcut.panel'/>
            <com.st.microxplorer.install.MXInstallPanel id='install.panel'/>
            <com.st.microxplorer.install.MXFinishPanel id='finish.panel'/>
        </AutomatedInstallation>"
    
    echo -n $AUTO_INSTALL_XML > auto-install.xml

    # create script to cd and run STM32CubeMX
    echo -n $'#!/bin/bash \n\n ' > cuberunner.sh
    echo -n "pushd /opt/STM32CubeMX_$CUBE_VERSION && ./STM32CubeMX && popd" >> cuberunner.sh
    
    cp $CURR_DIR/stm32cubemx-6.2.0/* .
    cat x* > en.stm32cubemx-lin_v6-2-0.zip

    if ! unzip $CUBE_ZIP_FILENAME ; then
        echo "##############################################################"
        echo "Error: Installing CubeMX failed"
        echo "Could not unzip $CUBE_ZIP_FILENAME"
        echo "##############################################################"
        exit 1
    fi
    
    sudo java -jar ./SetupSTM32CubeMX-$CUBE_VERSION auto-install.xml
    sudo cp ./cuberunner.sh /opt/STM32CubeMX_$CUBE_VERSION/cuberunner.sh

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
